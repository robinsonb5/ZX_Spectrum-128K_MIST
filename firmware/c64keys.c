#include <stdio.h>

#include "interrupts.h"
#include "c64keys.h"
#include "keyboard.h"
#include "ps2.h"
#include "user_io.h"
#include "spi.h"
#include "menu.h"
#include "timer.h"

#define QUAL_SPECIAL 0x8000	/* Key emits a different keycode if shift is held */
#define QUAL_LAYERKEY 0x4000 /* This key causes layers to switch */
#define QUAL_ALTLAYER 0x2000 /* Last keydown event happened while ctrl was held */
#define QUAL_LSHIFT 0x100
#define QUAL_RSHIFT 0x200
#define QUAL_CTRL 0x400
#define QUAL_BLOCKLAYER 0x800	/* R Amiga key or R Alt key - used to cancel layer shifting with quals */
#define QUAL_MASK 0x7f00

/*
Need to support multiple layers.
Ideally want shift keypresses to transfer between layers.
Need to make the layer sticky when a key is pressed, so the
key release event always matches the key press event.
The keytable currently contains 32-bit ints so plenty of room
for storing layer details in the map.
If just two layers will be sufficient, could simply store a second
entry shifted left 16 bits, accessed when the commodore key is held.
Use Run/Stop as an "Fn" key
*/


struct c64keyboard c64keys;


struct keyspecial
{
	unsigned char unshifted,shifted;
};

struct keyspecial specialtable[]=
{
	{KEY_RIGHTARROW,KEY_LEFTARROW},	/* cursor keys */
	{KEY_DOWNARROW,KEY_UPARROW},
	{KEY_F1,KEY_F2},	/* F keys */
	{KEY_F3,KEY_F4},
	{KEY_F5,KEY_F6},
	{KEY_F7,KEY_F8}
};

#define LAYER(x,y) ((x&0xffff)<<16)|(y)

#define KEY_MENUKEY 0xff

unsigned int c64keytable[]=
{
	LAYER(KEY_PRTSCRN,KEY_BACKSPACE), /* $00	Inst/Del */
	LAYER(KEY_NKENTER,KEY_ENTER), /* $01	Return */
	LAYER(QUAL_BLOCKLAYER|KEY_RCTRL,QUAL_SPECIAL|0), /* $02	Crsr l/r	- special handling needed */
	LAYER(KEY_F11,QUAL_SPECIAL|5), /* $03	F7/F8 */ /* FIXME - this one scancode exceeds 0x7f */
	LAYER(KEY_F9,QUAL_SPECIAL|2), /* $04	F1/F2 */
	LAYER(KEY_F10,QUAL_SPECIAL|3), /* $05	F3/F4 */
	QUAL_SPECIAL|4, /* $06	F5/F6 */
	LAYER(QUAL_BLOCKLAYER|KEY_ALTGR,QUAL_SPECIAL|1), /* $07	Crsr u/d	- special handling needed */

	LAYER(KEY_F3,KEY_3), /* $08	3 */
	KEY_W, /* $09	W */
	KEY_A, /* $0A	A */
	LAYER(KEY_F4,KEY_4), /* $0B	4 */
	KEY_Z, /* $0C	Z */
	KEY_S, /* $0D	S */
	KEY_E, /* $0E	E */
	QUAL_LSHIFT|KEY_LSHIFT, /* $0F	Left Shift - special handling needed */

	LAYER(KEY_F5,KEY_5), /* $10	5 */
	KEY_R, /* $11	R */
	KEY_D, /* $12	D */
	LAYER(KEY_F6,KEY_6), /* $13	6 */
	KEY_C, /* $14	C */
	KEY_F, /* $15	F */
	KEY_T, /* $16	T */
	KEY_X, /* $17	X */

	LAYER(KEY_NK7,KEY_7), /* $18	7 */
	KEY_Y, /* $19	Y */
	KEY_G, /* $1A	G */
	LAYER(KEY_NK8,KEY_8), /* $1B	8 */
	KEY_B, /* $1C	B */
	KEY_H, /* $1D	H */
	LAYER(KEY_NK4,KEY_U), /* $1E	U */
	KEY_V, /* $1F	V */

	LAYER(KEY_F9,KEY_9), /* $20	9 */
	LAYER(KEY_NK5,KEY_I), /* $21	I */
	LAYER(KEY_NK1,KEY_J), /* $22	J */
	LAYER(KEY_F10,KEY_0), /* $23	0 */
	LAYER(KEY_NK0,KEY_M), /* $24	M */
	LAYER(KEY_NK2,KEY_K), /* $25	K */
	LAYER(KEY_NK6,KEY_O), /* $26	O */
	KEY_N, /* $27	N */

	KEY_EQUALS, /* $28	+ */
	LAYER(KEY_NKASTERISK,KEY_P), /* $29	P */
	LAYER(KEY_NK3,KEY_L), /* $2A	L */
	KEY_MINUS, /* $2B	− */
	LAYER(KEY_NKPOINT,KEY_PERIOD), /* $2C	> */
	LAYER(KEY_NKMINUS,KEY_SEMICOLON), /* $2D	[ */
	LAYER(KEY_NUMLOCK,KEY_LEFTBRACE), /* $2E	@ */
	KEY_COMMA, /* $2F	< */
	
	KEY_BACKSLASH, /* $30	£ */
	LAYER(KEY_NKASTERISK,KEY_RIGHTBRACE), /* $31	* */
	KEY_APOSTROPHE, /* $32	] */
	KEY_DELETE, /* $33	Clr/ Home */
	QUAL_RSHIFT|KEY_RSHIFT, /* $34	Right shift - special handling needed */
	LAYER(KEY_HASH,KEY_TICK), /* $35	= */
	KEY_HASH, /* $36	↑ */
	LAYER(KEY_NKPLUS,KEY_SLASH), /* $37	? */

	LAYER(KEY_F1,KEY_1), /* $38	1 */
	LAYER(KEY_F12,KEY_ESC), /* $39	← */
	LAYER(KEY_TAB,KEY_TAB), /* $3A	Control */
	LAYER(KEY_F2,KEY_2), /* $3B	2 */
	KEY_SPACE, /* $3C	Space */
	QUAL_CTRL|KEY_LCTRL, /* $3D	Commodore */
	KEY_Q, /* $3E	Q */
	QUAL_LAYERKEY  /* $3F	Run/Stop */
};

#if 0
static void c64_rb_write(struct c64keyboard *r,int in)
{
//	printf("%x ",in);
	r->outbuf[r->out_cpu]=in;
	r->out_cpu=(r->out_cpu+1) & (C64KEY_RINGBUFFER_SIZE-1);
}
#endif

void c64keyboard_write(struct c64keyboard *r,int in)
{
	/* We disable interrupts here since we're adding key events both
	   to the controller's incoming PS/2 buffer */
	DisableInterrupts();

	if(in&0x80)
		PS2KeyboardReceive(0xe0);
	if(in&0x100)
		PS2KeyboardReceive(0xf0);
	PS2KeyboardReceive(in&0x7f);	

	EnableInterrupts();
}


/* Pull this into a separate function since it's not safe to do SPI transactions in an interrupt */
#if 0
void sendc64keys()
{
	static int sendtime=0;
	DisableInterrupts();
	if(CheckTimer(sendtime) && c64keys.out_hw!=c64keys.out_cpu)
	{
		char key=c64keys.outbuf[c64keys.out_hw];
		sendtime=GetTimer(((key==0xe0) || (key==0xf0)) ? 4 : 15 ); /* Delay between events, to ensure shift/cursor is recognised. */
		EnableIO();
		SPI(UIO_KEYBOARD);
		SPI(key);
		DisableIO();
		c64keys.out_hw=(c64keys.out_hw+1) & (C64KEY_RINGBUFFER_SIZE-1);
	}
	EnableInterrupts();
}
#endif

/* We don't want to poll the keyboard too quickly or we'll undo the filtering required to detect joystick port interference. */
#define FRAMETIME 0x20

void handlec64keys()
{
	int i;
	static int time=0;
	static int sendtime=0;
	int count=0;
	int idx=63;
	int nextframe=(c64keys.frame+2)%6;
	int prevframe=(c64keys.frame+4)%6;

	unsigned int aa;
	unsigned int ad;

	if(!CheckTimer(time))
		return;
	time=GetTimer(FRAMETIME);

	for(i=0;i<2;++i)
	{
		unsigned int t=HW_KEYBOARD(REG_KEYBOARD_WORD0+4*i);
		c64keys.keys[c64keys.frame+i]=t;
		while(t)	/* Count the number of set bits */
		{
			t&=t-1;
			++count;
		}
	}

	/* 	Very unlikely that more than four keys are depressed, so if that happens
		we're probably looking at interference from the joystick port. */
	if(count>=60)
	{
		for(i=0;i<2;++i)
		{
			int j;

			/*  Keystroke detection:
				The C64 keyboard shares lines with joystick port 1, so reading the keyboard robustly
				is difficult.  The above test filters out reading where a whole row is shorted to ground
				by the joystick, but we can still get transients if the joystick event happens while scanning the keyboard.
				We therefore filter the data with an edge-detection:

				nextframe contains the data from two frames ago, prevframe and c64frame are self-explanatory.
				Edge detection works like this:  (A is nextframe, B is prevframe, C is c64frame)
				A B C	Edge?
				0 0 0	0	- key is held down
				0 0 1	0	- key release
				0 1 0	0	- transient
				0 1 1	1	- key release, stable
				1 0 0	1	- key press, stable
				1 0 1	0	- transient keypress
				1 1 0	0	- keypress, not yet verified
				1 1 1	0	- key up.

				edge = (A^B) & (A^C)
			*/

			unsigned int changed=(c64keys.keys[nextframe+i]^c64keys.keys[prevframe+i])
										&(c64keys.keys[nextframe+i]^c64keys.keys[c64keys.frame+i]);
			unsigned int status=c64keys.keys[c64keys.frame+i];
			for(j=0;j<32;++j)
			{
				--idx;
				if(changed>>31)
				{
					int code=63-(i*32+j);	/* Fetch Amiga scancode for this key */
					int amicode;
					int amiqualup=0;
					int amiqualdown=0;
					int keyup=status>>31;

					c64keys.active=1;
					code=((code<<3)|(code>>3))&63;	/* bit numbers are transposed compared with c64 scancodes */
					amicode=c64keytable[code];
//					printf("Event on key %d -> \n",code,amicode);

					/* Has the run/stop key (acting as Fn) been pressed? */
					if(amicode&QUAL_LAYERKEY)
					{
						if(keyup)	/* Key up? */
						{
							c64keys.layer=0;
							c64keys.layered=0;
						}
						else
						{
							c64keys.layer=1;
							if(!c64keys.layered)
							{
								c64keyboard_write(&c64keys,KEY_CAPSLOCK);
								c64keyboard_write(&c64keys,KEY_CAPSLOCK|0x100);
							}								
						}
					}
					else
					{
						/* If this is a keyup event, make sure it happens on the same layer as the corresponding keydown. */
						if(keyup) /* key up? */
						{
							if(amicode&QUAL_ALTLAYER) /* Was the keydown on the alternative layer? */
								amicode>>=16;
							c64keytable[code]&=~QUAL_ALTLAYER;
						}
						/* Otherwise generate a keydown for the appropriate layer */
						else if(c64keys.layer && (amicode>>16))
						{
							/* Cancel the second layer for non-qualifier keys if qualifiers are pressed */
						 	if (!(c64keys.qualifiers&QUAL_BLOCKLAYER) || ((amicode>>16)&QUAL_BLOCKLAYER))
							{
								c64keytable[code]|=QUAL_ALTLAYER;
								amicode>>=16;
							}
						}

						if(amicode&QUAL_SPECIAL)
						{
							/* If the key requires special handling, cancel any shifting before sending the key code
								unless both shift keys are down */
							switch(c64keys.qualifiers&(QUAL_LSHIFT|QUAL_RSHIFT))
							{
								case 0:
									amicode=specialtable[amicode&0xff].unshifted;
									break;
								case QUAL_LSHIFT:
									amicode=specialtable[amicode&0xff].shifted;
									if(keyup)
										amiqualdown=KEY_LSHIFT;
									else
										amiqualup=KEY_LSHIFT|0x100;
									break;
								case QUAL_RSHIFT:
									amicode=specialtable[amicode&0xff].shifted;
									if(keyup)
										amiqualdown=KEY_RSHIFT;
									else
										amiqualup=KEY_RSHIFT|0x100;
									break;
								default:
									amicode=specialtable[amicode&0xff].shifted;
									break;
							}
						}
						if(keyup)
						{
							c64keys.qualifiers&=(~amicode)&QUAL_MASK;
							amicode&=0xff;
							amicode|=0x100; /* Key up */
						}
						else
						{
							c64keys.qualifiers|=amicode&QUAL_MASK;
							amicode&=0xff;
						}
						if(amiqualup)
							c64keyboard_write(&c64keys,amiqualup);
						c64keyboard_write(&c64keys,amicode);
						if(amiqualdown)
							c64keyboard_write(&c64keys,amiqualdown);
					}
				}
				changed<<=1;
				status<<=1;
			}
		}
		c64keys.frame=nextframe;
	}
}

void initc64keys()
{
	int i;
	for(i=0;i<6;++i)
		c64keys.keys[i]=0xffffffff;
	c64keys.frame=0;
	c64keys.layer=0;
	c64keys.qualifiers=0;
}

