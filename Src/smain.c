#include "main.h"
#include "ili9341.h"
#include "fatfs.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
void mprintf(const char *fmt, ...);
int  mp3_play(char* filename);
char* getFileName();
int   getModeFile();
void  DirtyMemorySet();
void tapeSetTime();
#include "z80.h"
uint8_t   RAMP[4][0x4000];
uint8_t   CRAM[4][0x4000]  __attribute__((section(".ccmram")));
uint8_t * const RAM[8] =
{
	&RAMP[0][0],
	&RAMP[1][0],
	&RAMP[2][0],
	&RAMP[3][0],
	&CRAM[0][0],
	&CRAM[1][0],
	&CRAM[2][0],
	&CRAM[3][0]
};
#include "main_ROM.h"
#include "128-0.h"
#include "128-1.h"
const uint8_t*   ROM[]=
{
	&fuse_roms_128_0_rom[0],
	&fuse_roms_128_1_rom[0],
	&fuse_roms_48_rom[0]
};

uint8_t * getRamBuffer0x4000(int k)
{
	return RAM[k];
}
void tape_close();
int  tape_open(char* fileName);
void tapeStart();
void tapeStop();
int  getTapeState();
int  tapeBit();

extern int32_t tstates;
extern int32_t interrupts_enabled_at;

struct sSINCLAIR_FLAGS
{
	int             flag128;
	uint16_t 	    border;
	uint8_t 	    borderArray[ILI9341_LCD_PIXEL_WIDTH];
	int 			portMap   ;
	int 			ram_bank ;
	int 			rom_bank ;
	int 			shadow_bank;
	int 			T69888;
	int 			bColor;
	int     		joystickMode;
	uint8_t			IsPressed[8];
	uint8_t			Kempston;  //public
	u16 			prevKey;
	uint32_t        frameCounter;
	int             tapeSpeed;
} SINCLAIR_FLAGS;

void clearKeys()
{
	SINCLAIR_FLAGS.prevKey = 0xffff;
	SINCLAIR_FLAGS.IsPressed[0]= 0xff;
	SINCLAIR_FLAGS.IsPressed[1]= 0xff;
	SINCLAIR_FLAGS.IsPressed[2]= 0xff;
	SINCLAIR_FLAGS.IsPressed[3]= 0xff;
	SINCLAIR_FLAGS.IsPressed[4]= 0xff;
	SINCLAIR_FLAGS.IsPressed[5]= 0xff;
	SINCLAIR_FLAGS.IsPressed[6]= 0xff;
	SINCLAIR_FLAGS.IsPressed[7]= 0xff;
}

void init_Sinclair()
{
	SINCLAIR_FLAGS.bColor = 1;
	SINCLAIR_FLAGS.joystickMode = 0;
	SINCLAIR_FLAGS.Kempston = 0;
	SINCLAIR_FLAGS.prevKey = 0xffff;
	clearKeys();

}

int getT69888()
{
	return SINCLAIR_FLAGS.T69888;
}
uint32_t getFrameCounter()
{
	return SINCLAIR_FLAGS.frameCounter;
}


uint8_t    readByte(uint16_t adress);
void       writeByte(uint16_t adress,uint8_t data);
#define  VIDEO_RAM (&RAM[SINCLAIR_FLAGS.shadow_bank][0x0])
#define  ATTR_RAM  (&RAM[SINCLAIR_FLAGS.shadow_bank][0x1800])
uint8_t  ATTR_RAM_MOD[(0x5B00-0x5800)>>3];
int getFileType(char* name)
{
	int type = T_UNKNOWN;
	if(strcasestr(name,".tap"))
	{
		type = T_TAP;
	}
	else if(strcasestr(name,".z80"))
	{
		type = T_Z80;
	}
	else if(strcasestr(name,".mp3"))
	{
		type = T_MP3;
	}
	return type;
}



uint16_t scanJSK();
void initRam()
{
	SINCLAIR_FLAGS.shadow_bank   = 5; // or 7
	SINCLAIR_FLAGS.rom_bank      = 0;
	SINCLAIR_FLAGS.ram_bank      = 0;
	SINCLAIR_FLAGS.portMap       = 0;
	SINCLAIR_FLAGS.T69888        = 70908;
	memset(SINCLAIR_FLAGS.borderArray,0,sizeof(SINCLAIR_FLAGS.borderArray));
	SINCLAIR_FLAGS.border = 0;
	SINCLAIR_FLAGS.frameCounter = 0;

	if(!SINCLAIR_FLAGS.flag128)
	{
		SINCLAIR_FLAGS.rom_bank = 2;
		SINCLAIR_FLAGS.portMap  = 0x20;
		SINCLAIR_FLAGS.T69888   = 69888;
	}
	else
	{

	}
}
void clearAttr()
{
	int k;
	for(k=0;k<((0x5B00-0x5800)>>3);k++)
	{
		ATTR_RAM_MOD[k] = 0;
	}
}
void setAttr()
{
	int k;
	for(k=0;k<((0x5B00-0x5800)>>3);k++)
	{
		ATTR_RAM_MOD[k] = 0xff;
	}
}

void setMemPages(uint8_t val)
{
	if(SINCLAIR_FLAGS.portMap&0x20)
	{
		// locked
	}
	else
	{
		SINCLAIR_FLAGS.portMap   =  val;
		SINCLAIR_FLAGS.rom_bank 		=  (val&0x10)?1:0;
		int new_ram_bank 	=  val&0x7; 
		int new_shadow_bank = (val&0x8)?7:5;
		if(new_shadow_bank!=SINCLAIR_FLAGS.shadow_bank|| ((new_ram_bank!=SINCLAIR_FLAGS.ram_bank)&&((new_ram_bank == 5) || (SINCLAIR_FLAGS.ram_bank == 5))))
		{
			setAttr();
		}
		SINCLAIR_FLAGS.shadow_bank = new_shadow_bank;
		SINCLAIR_FLAGS.ram_bank = new_ram_bank;
	}
}
u8  peek(uint16_t addr) 
{
	u8 res;
	//~ int page = addr>>14u;
	switch(addr>>14u)
	{
		case 0: 
		{
			//~ [(portMap>>5)&1]; 
			res = ROM[SINCLAIR_FLAGS.rom_bank][addr];
		}
		break;
		case 1:  
		{
				res = RAM[5][addr-0x4000]; 
		}
		break;
		case 2:  	res = RAM[2][addr-0x8000]; break;
		case 3:  
		default:
				res = RAM[SINCLAIR_FLAGS.ram_bank][addr-0xc000]; break;
	};
	return res;
}
void  poke(uint16_t addr,uint8_t value) 
{
	switch(addr>>14u)
	{
		case 0: 
		{
			//~ [(portMap>>5)&1]; 
			//~ res = ROM[addr];  
			// write to rom??
		}
		break;
		case 1:  
		{
			tstates+=3;
			RAM[5][addr-0x4000] = value;
			if(SINCLAIR_FLAGS.shadow_bank==5)
			{
				if(addr<0x5B00)
				{
					uint16_t pa = (addr<0x5800)?((addr&0x00ff) | (addr&0x1800)>>3):(addr-0x5800);
					ATTR_RAM_MOD[pa>>3] |= (1<<(pa&0x3));
				}
			}
		}
		break;
		case 2:  RAM[2][addr-0x8000] = value; break;
		case 3:  
			if(SINCLAIR_FLAGS.shadow_bank==7)
			{
				tstates+=3;
				if(addr<(0x5B00+0x8000))
				{
					uint16_t pa = (addr<(0x5800+0x8000))?((addr&0x00ff) | (addr&0x1800)>>3):(addr-(0x5800+0x8000));
					ATTR_RAM_MOD[pa>>3] |= (1<<(pa&0x3));
				}
			}
			if (SINCLAIR_FLAGS.shadow_bank==5 && SINCLAIR_FLAGS.ram_bank==5)
			{
				poke(addr-0x8000,value);
			}
			else
			{
				RAM[SINCLAIR_FLAGS.ram_bank][addr-0xc000] = value;
			}
			break;
	};
}

#if 0
//adddress bus:	//8	//9	//10	//11	//12	//13	//14	//15		
const int keyMatrix[5][8] = {
	{ CAPS_SHIFT	,'A'	,'Q'	,'1'	,'0'	,'P'	,ENTER	,SPACE		},	// d0
	{ 'Z'		,'S'	,'W'	,'2'	,'9'	,'O'	,'L'	,SYMB_SHIFT	},	// d1
	{ 'X'		,'D'	,'E'	,'3'	,'8'	,'I'	,'K'	,'M'		},	// d2
	{ 'C'		,'F'	,'R'	,'4'	,'7'	,'U'	,'J'	,'N'		},	// d3
	{ 'V'		,'G'	,'T'	,'5'	,'6'	,'Y'	,'H'	,'B'		}	// d4	
	};

const int myMatrix[3][4] = {
	{ 0x40		,' '	,0x10	,' '	},	// d
	{ ' '		,0x4	,' '	,0x8	},	// d
	{ 0x20		,' '	,0x1	,0x2	}	// d	
	};

#endif	

#define SPACE          	(((15-8)<<8)|0)
#define SYMB_SHIFT     	(((15-8)<<8)|1)
#define KEY_M          	(((15-8)<<8)|2)
#define KEY_N          	(((15-8)<<8)|3)
#define KEY_B          	(((15-8)<<8)|4)

#define ENTER         	(((14-8)<<8)|0)
#define KEY_L          	(((14-8)<<8)|1)
#define KEY_K          	(((14-8)<<8)|2)
#define KEY_J          	(((14-8)<<8)|3)
#define KEY_H          	(((14-8)<<8)|4)

#define KEY_P         	(((13-8)<<8)|0)
#define KEY_O          	(((13-8)<<8)|1)
#define KEY_I          	(((13-8)<<8)|2)
#define KEY_U          	(((13-8)<<8)|3)
#define KEY_Y          	(((13-8)<<8)|4)

#define KEY_0         	(((12-8)<<8)|0)
#define KEY_9          	(((12-8)<<8)|1)
#define KEY_8          	(((12-8)<<8)|2)
#define KEY_7          	(((12-8)<<8)|3)
#define KEY_6          	(((12-8)<<8)|4)

#define KEY_1         	(((11-8)<<8)|0)
#define KEY_2          	(((11-8)<<8)|1)
#define KEY_3          	(((11-8)<<8)|2)
#define KEY_4          	(((11-8)<<8)|3)
#define KEY_5          	(((11-8)<<8)|4)

#define KEY_Q         	(((10-8)<<8)|0)
#define KEY_W          	(((10-8)<<8)|1)
#define KEY_E          	(((10-8)<<8)|2)
#define KEY_R          	(((10-8)<<8)|3)
#define KEY_T          	(((10-8)<<8)|4)

#define KEY_A         	(((9-8)<<8)|0)
#define KEY_S          	(((9-8)<<8)|1)
#define KEY_D          	(((9-8)<<8)|2)
#define KEY_F          	(((9-8)<<8)|3)
#define KEY_G          	(((9-8)<<8)|4)

#define CAPS_SHIFT 		(((8-8)<<8) |0)
#define KEY_Z          	(((8-8)<<8)|1)
#define KEY_X          	(((8-8)<<8)|2)
#define KEY_C          	(((8-8)<<8)|3)
#define KEY_V          	(((8-8)<<8)|4)

inline void setRes(u8 adr,u8 bit,int setResFlag)
{
	if(setResFlag)
	{
		SINCLAIR_FLAGS.IsPressed[adr]|= 1u<<bit;
	}
	else
	{
		SINCLAIR_FLAGS.IsPressed[adr]&= ~(1u<<bit);
	}
		
}
inline void setResK(u16 key,int setResFlag)
{
	setRes(key>>8,key&0xff,setResFlag);
}

struct SINCL_KEY_EVENT
{
	unsigned code   :16;
	unsigned pressed:1;
	unsigned wait_for_next:15;
};
#define   KEY_QUE_SIZE 32

struct SINCL_KEY_EVENT KEY_QUE[KEY_QUE_SIZE];

int KEY_QUE_head;
int KEY_QUE_tail;
void KEY_QUE_init()
{
	KEY_QUE_head = 0;
	KEY_QUE_tail = 0;
}
int32_t KEY_QUE_size()
{
	return (KEY_QUE_head+KEY_QUE_SIZE-KEY_QUE_tail)%KEY_QUE_SIZE;
}

void KEY_QUE_put(unsigned code,unsigned pressed,unsigned wait_for_next)
{
	struct SINCL_KEY_EVENT ev;
	ev.code = code;
	ev.pressed = pressed;
	ev.wait_for_next = wait_for_next;
	if(KEY_QUE_size()<KEY_QUE_size-1)
	{
		KEY_QUE[KEY_QUE_head] = ev;
		KEY_QUE_head = (KEY_QUE_head+1)% KEY_QUE_SIZE;
	}
}
void KEY_QUE_pop()
{
	if(KEY_QUE_size())
	{
		KEY_QUE_tail = (KEY_QUE_tail+1)% KEY_QUE_SIZE;
	}
}

char *joyMap[] =
{
		"76580 ",
		"98670 ",
		"43125 ",
		"AZQW 0",
		"^v<>! "
};
char* JS_LINES[]=
{
		"  Cursor  ",
		"Sinclair 1",
		"Sinclair 2",
		"  AZQW    ",
		" Kempston "
};
#define joyMapSize (sizeof(joyMap)/sizeof(joyMap[0]))
u16 keyScan()
{
	u16 res =~scanJSK();//0;//((KEYB_0_GPIO_Port->IDR>>3)&0b1111111);

//	res = 0b1111111;
//	if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin)==GPIO_PIN_RESET)
//	{
//		res&=~0x20;
//	}
//	if(sample_touch())
//	{
//		res&=~2;
//		mprintf("space \r\n");
//	}

	if(res != SINCLAIR_FLAGS.prevKey)
	{
		SINCLAIR_FLAGS.prevKey = res ;
		SINCLAIR_FLAGS.Kempston = 0;
	if(SINCLAIR_FLAGS.joystickMode==0) //cursor
	{
//cursor joy matrix
//	up     - 7
//	down   - 6
//	left   - 5
//	right  - 8
// 	fire 	 0
//	special- " "
		setResK(KEY_7,res&K_UP);
		setResK(KEY_6,res&K_DOWN);
		setResK(KEY_5,res&K_LEFT);
		setResK(KEY_8,res&K_RIGHT);
		
		setResK(KEY_0,res&K_FIRE);
		
		setResK(SPACE,res&K_SPACE);  //space
//		setRes(12-8,3,res&K_UP);
//		setRes(12-8,4,res&K_DOWN);
//		setRes(11-8,4,res&K_LEFT);
//		setRes(12-8,2,res&K_RIGHT);
//
//		setRes(12-8,0,res&K_FIRE);
//
//		setRes(15-8,0,res&K_TOUCH);  //space
	}
	else if((SINCLAIR_FLAGS.joystickMode)==1)
	{
//sinclair i1 joy matrix
//	up     - 9
//	down   - 8
//	left   - 6
//	right  - 7
// 	fire 	 0
		setResK(KEY_9,res&K_UP);
		setResK(KEY_8,res&K_DOWN);
		setResK(KEY_6,res&K_LEFT);
		setResK(KEY_7,res&K_RIGHT);
		
		setResK(KEY_0,res&K_FIRE);
		
		setResK(SPACE,res&K_SPACE);  //space

//		setRes(12-8,1,res&K_UP);
//		setRes(12-8,2,res&K_DOWN);
//		setRes(12-8,4,res&K_LEFT);
//		setRes(12-8,3,res&K_RIGHT);
//
//		setRes(12-8,0,res&K_FIRE);
//
//		setRes(15-8,0,res&K_TOUCH);  //space
	}
	else if((SINCLAIR_FLAGS.joystickMode)==2)
	{
//sinclair j2 joy matrix
		setResK(KEY_4,res&K_UP);
		setResK(KEY_3,res&K_DOWN);
		setResK(KEY_1,res&K_LEFT);
		setResK(KEY_2,res&K_RIGHT);
		
		setResK(KEY_5,res&K_FIRE);
		
		setResK(SPACE,res&K_SPACE);  //space

	}
	else if((SINCLAIR_FLAGS.joystickMode)==3)       //keyboard
	{
//sinclair key matrix
//	up     - a
//	down   - z
//	left   - q
//	right  - w
// 	fire 	 space
		setResK(KEY_A,res&K_UP);
		setResK(KEY_Z,res&K_DOWN);
		setResK(KEY_Q,res&K_LEFT);
		setResK(KEY_W,res&K_RIGHT);
		
		setResK(SPACE,res&K_FIRE);
		
		setResK(SPACE,res&K_SPACE);  //space

	}
	else if(SINCLAIR_FLAGS.joystickMode==4)
	{
// kempston joy on port 31
//	up     - 0x8
//	down   - 0x4
//	left   - 0x2
//	right  - 0x1
// 	fire 	 0x10
//
		SINCLAIR_FLAGS.Kempston = ~(((res&K_UP)?0x8:0)|((res&K_DOWN)?0x4:0)|((res&K_LEFT)?0x2:0)|((res&K_RIGHT)?0x1:0)|((res&K_FIRE)?0x10:0));
	}
	}
	return res^0b1111111;
}
int keyDelayCounter = 0;
uint16_t lastKeyEvent = 0;
u8 in(u16 port)
{
	u8 input=0xff;
	
	if ((port&0x00FF)==31)
	{
		keyScan();
		return SINCLAIR_FLAGS.Kempston;
	}
	if ((port&0x00FF)==0xFE)
	{
		if (keyDelayCounter>0) 
		{
			keyDelayCounter--;
		}
		if(keyDelayCounter==0)
		{
			if(KEY_QUE_size())
			{
				struct SINCL_KEY_EVENT ev = KEY_QUE[KEY_QUE_tail];
				setRes(ev.code>>8,ev.code&0xff,!ev.pressed);
				mprintf("ev.code %x ,ev.pressed %x ,ev.wait_for_next %x %d\r\n",ev.code,ev.pressed,ev.wait_for_next,KEY_QUE_size());
				KEY_QUE_pop();
				keyDelayCounter = ev.wait_for_next;
			}
			else
			{
				keyScan();
			}
		}

		int bit;
		for(bit = 0;bit<8; bit++)
		{
			if(!(port&(1<<(bit+8))))
			{
			   input &= SINCLAIR_FLAGS.IsPressed[bit];
			}
		}
		if(SINCLAIR_FLAGS.tapeSpeed!=0)
		{
			if(tapeBit()==0)
			{
				input&=0x10111111;
			}
		}
		tstates += 3;
		return (input);
	}
	else
	{
		return (0xff);
	}
	

}
#define SOUND_EVENT_ARRAY_SIZE 1024

union a_event
{
	struct
	{
		int    		t    :19;
		unsigned 	ay   :1;
		unsigned 	cmd  :4;
		unsigned 	data :8;
	} u;
	int32_t         event;
};

union a_event aSOUND_EVENTS[SOUND_EVENT_ARRAY_SIZE];
int aSOUND_EVENTS_head = 0;
int aSOUND_EVENTS_tail = 0;

void aSOUND_EVENTS_init()
{
	aSOUND_EVENTS_head = 0;
	aSOUND_EVENTS_tail = 0;
}

int32_t aSOUND_EVENTS_size()
{
	return ((aSOUND_EVENTS_head+SOUND_EVENT_ARRAY_SIZE)-aSOUND_EVENTS_tail)%SOUND_EVENT_ARRAY_SIZE;
}

void aSOUND_EVENTS_put(union a_event ev)
{
	if(aSOUND_EVENTS_size()<SOUND_EVENT_ARRAY_SIZE-1)
	{
		aSOUND_EVENTS[aSOUND_EVENTS_head] = ev;
		aSOUND_EVENTS_head = (aSOUND_EVENTS_head+1)% SOUND_EVENT_ARRAY_SIZE;
	}
}
void aSOUND_EVENTS_pop()
{
	if(aSOUND_EVENTS_size())
	{
		aSOUND_EVENTS_tail = (aSOUND_EVENTS_tail+1)% SOUND_EVENT_ARRAY_SIZE;
	}
}


volatile int soundBitState = 0;
//make  20msec sound buffer ~ 44100/50 = 882 samples
void initTick();

#define AY_REGISTERS 16

typedef struct ayinfo {
  int current_register;
  uint8_t registers[ AY_REGISTERS ];
} ayinfo,ayinfo_delayed;


ayinfo ay_z80;
static const uint8_t ay_mask[ AY_REGISTERS ] = {

  0xff, 0x0f, 0xff, 0x0f, 0xff, 0x0f, 0x1f, 0xff,
  0x1f, 0x1f, 0x1f, 0xff, 0xff, 0x0f, 0xff, 0xff,

};
void sound_ay_write(uint8_t current, uint8_t data,int32_t tstates_ay)
{
	union a_event r;

	r.u.t   = tstates_ay;
	r.u.ay =  1;
	r.u.data = data;
	r.u.cmd = current;
	//~ if(flag128)
	{
		aSOUND_EVENTS_put(r);
	}
}

void ay_reset()
{
	ayinfo *ay = &ay_z80;
	ay->current_register = 0;
	memset( ay->registers, 0, sizeof( ay->registers ) );
	initTick();
}
uint8_t ay_read()
{
	int current;
	const uint8_t port_input = 0xbf;
	current = ay_z80.current_register;
  if( current == 14 ) {
    if(ay_z80.registers[7] & 0x40)
      return (port_input & ay_z80.registers[14]);
    else
      return port_input;
    }
/* R15 is simpler to do, as the 8912 lacks the second I/O port, and
     the input-mode input is always 0xff */
  if( current == 15 && !( ay_z80.registers[7] & 0x80 ) )
    return 0xff;

  /* Otherwise return register value, appropriately masked */
  return ay_z80.registers[ current ] & ay_mask[ current ];


}
void ay_write_command(uint8_t cmd)
{
	ay_z80.current_register = (cmd & 0x0f);
}
void ay_write_data(uint8_t data)
{
  int current;

  current = ay_z80.current_register;

  ay_z80.registers[ current ] = data & ay_mask[ current ];
  sound_ay_write( current, data, tstates );
  //~ if( psg_recording ) psg_write_register( current, b );

  if( current == 14 )
  {
	 // printer_serial_write( b );!!!
  }
}
#define NUM_TONES 5
  static const int levels[16] = {
    0x0000, 0x0385, 0x053D, 0x0770,
    0x0AD7, 0x0FD5, 0x15B0, 0x230C,
    0x2B4C, 0x43C1, 0x5A4B, 0x732F,
    0x9204, 0xAFF1, 0xD921, 0xFFFF
  };
static int rng = 1;
struct soundModule
{
	uint8_t reg[16];
	int counters[NUM_TONES];
	int counters_period[NUM_TONES];

	int counters_state[NUM_TONES];
	int tick_inv[NUM_TONES];
	int pattern_nu;
	int count;
	int val;

}soundModule_sm;
#define ATTACK_BIT 		0b0100
#define ALTERNATE_BIT 	0b0010
#define HOLD_BIT 		0b0001
#define CONTINUE_BIT	0b1000

void incPattern()
{
	//~ int flgEnd = count>15;
	if(soundModule_sm.pattern_nu&CONTINUE_BIT)
	{
		soundModule_sm.val = soundModule_sm.count&0xf;
		if(((soundModule_sm.pattern_nu&ALTERNATE_BIT)&&(soundModule_sm.count&0x10))^(!(soundModule_sm.pattern_nu&ATTACK_BIT)))
		{
			soundModule_sm.val = 0xf-soundModule_sm.val;
		}
		if((soundModule_sm.pattern_nu&HOLD_BIT)&&(soundModule_sm.count>0xf))
		{
			soundModule_sm.val = ((soundModule_sm.pattern_nu^(soundModule_sm.pattern_nu<<1))&ATTACK_BIT)?0xf:0;
		}
	}
	else
	{
		soundModule_sm.val = (soundModule_sm.count>0xf)?0:((soundModule_sm.pattern_nu&ATTACK_BIT)?soundModule_sm.count:(0xf-soundModule_sm.count));
	}
	soundModule_sm.count++;
}
int initPattern(int _pattern_nu)
{
	soundModule_sm.count = 0;
	soundModule_sm.pattern_nu = _pattern_nu;
	incPattern();
	return 0;
}

void initTick()
{
	for(int k=0;k<NUM_TONES;k++)
	{
		soundModule_sm.counters[k] = 0;
		soundModule_sm.counters_period[k] = 0;
		soundModule_sm.counters_state[k] =1;
		soundModule_sm.tick_inv[k ] =0;
	}
	for(int k=0;k<16;k++)
	{
		soundModule_sm.reg[k ] =0;
	}
}
void clearTick()
{
 for(int k=0;k<NUM_TONES;k++)
 {
	 soundModule_sm.tick_inv[k ] = 0;
 }

}
int noise_toggle;
void incrTick(int tiks)
{
	//~ PRINT(tiks);

	for(int k=0;k<NUM_TONES;k++)
	{
		soundModule_sm.counters[k]+= tiks;
		int divider = soundModule_sm.counters_period[k];
		if(divider==0)
		{
			divider=1;
		}
		int cl = soundModule_sm.counters[k]/divider;
		if(cl&0x1)
		{
			soundModule_sm.counters_state[k] =  -soundModule_sm.counters_state[k];
		}
		soundModule_sm.counters[k]-=divider*cl;
		cl = cl&0x1f;
		if(k==NUM_TONES-2)
		{
			for(int j=0;j<cl;j++)
			{
				if( ( rng & 1 ) ^ ( ( rng & 2 ) ? 1 : 0 ) )
				noise_toggle = !noise_toggle;
				  if( rng & 1 )
				  {
					  rng ^= 0x24000;
				  }
				  rng >>= 1;
			}
		}
		if(k==NUM_TONES-1)
		{
			for(int j=0;j<cl;j++)
			{
				incPattern();
			}
		}
	}
//	for(int k=0;k<NUM_TONES;k++)
//	{
//		soundModule_sm.counters[k]+= tiks;
//		int divider = soundModule_sm.counters_period[k];
//		while(soundModule_sm.counters[k]>=divider)
//		{
//			soundModule_sm.counters[k]-=divider;
//			soundModule_sm.counters_state[k] =  -soundModule_sm.counters_state[k];
//			if(k==NUM_TONES-2)
//			{
//				if( ( rng & 1 ) ^ ( ( rng & 2 ) ? 1 : 0 ) )
//				noise_toggle = !noise_toggle;
//				  if( rng & 1 ) {
//				rng ^= 0x24000;
//				  }
//				  rng >>= 1;
//			}
//			if(k==NUM_TONES-1)
//			{
//				incPattern();
//			}
//			soundModule_sm.tick_inv[k]++;
//		}
//	}
	//~ int k =

}
//~ int volume[3];
int getVolPattern()
{
	return levels[soundModule_sm.val&0xf]/32;
}

int getVolume(int chan)
{
	//~ PRINT(int(reg[8+chan]));
	return (soundModule_sm.reg[8+chan]&0x10)?getVolPattern():((levels[soundModule_sm.reg[8+chan]&0xf])/32);
}
int noiseBit()
{
	return (noise_toggle&1)*2-1;
}
int getSign()
{
	int amp = 0;
	if(!SINCLAIR_FLAGS.flag128)
	{
		return amp;
	}
	//~ PRINT(amp);
	if((soundModule_sm.reg[7]&1)==0)
	{
		amp+= getVolume(0)*soundModule_sm.counters_state[0];
	}
	//~ PRINT(amp);
	if((soundModule_sm.reg[7]&2)==0)
	{
		amp+= getVolume(1)*soundModule_sm.counters_state[1];
	}
	//~ PRINT(amp);
	if((soundModule_sm.reg[7]&4)==0)
	{
		amp+= getVolume(2)*soundModule_sm.counters_state[2];
	}
	//~ PRINT(amp);
	if((soundModule_sm.reg[7]&8)==0)
	{
		amp+= getVolume(0)/2*noiseBit();
	}
	if((soundModule_sm.reg[7]&0x10)==0)
	{
		amp+= getVolume(1)/2*noiseBit();
	}
	if((soundModule_sm.reg[7]&0x20)==0)
	{
		amp+= getVolume(2)/2*noiseBit();
	}

	return amp;
}
#define RSCALE (16)
void execute(union a_event* ev)
{
	if(!ev->u.ay) return;
	soundModule_sm.reg[ev->u.cmd&0xf] = ev->u.data;
	//cout<<hex;
	switch(ev->u.cmd)
	{
		case 0:
		case 1:  soundModule_sm.counters_period[0] = ( (256*(int)(soundModule_sm.reg[1])) +((int)(soundModule_sm.reg[0])))*RSCALE ; break;
		case 2:
		case 3:  soundModule_sm.counters_period[1] = ((256*(int)(soundModule_sm.reg[3])) +((int)(soundModule_sm.reg[2])))*RSCALE ; break;
		case 4:
		case 5:  soundModule_sm.counters_period[2] = ((256*(int)(soundModule_sm.reg[5])) +((int)(soundModule_sm.reg[4])))*RSCALE ; break;
		case 6:  soundModule_sm.counters_period[3] = (int)(soundModule_sm.reg[6])*RSCALE ;  break;
		case 7:   break;
		case 8:   break;
		case 9:   break;
		case 10: break;
		case 11:
		case 12: soundModule_sm.counters_period[4] = ((256*(int)(soundModule_sm.reg[12])) +((int)(soundModule_sm.reg[11])))*RSCALE; break;
		case 13: initPattern(soundModule_sm.reg[13]); break;
		case 14: break;
		case 15: break;
	};
}


void soundEvents(int event)
{
	union a_event r;
	r.u.t    = tstates;
	r.u.ay   = 0;
	r.u.data = event;
	r.u.cmd  = 0;
	aSOUND_EVENTS_put(r);
}

static int32_t fuller_next = 0;
static int32_t fuller = 0;
void push_pair(uint16_t,uint16_t);
int waitFor();

#include <math.h>
int  sound_size();
void proccesSoundEvents_time()
{
	// wait interrupt for free buffer
	//  get last time schedule event;
	soundEvents(100);

	union a_event first = aSOUND_EVENTS[aSOUND_EVENTS_tail];


	int32_t currentTime  =  first.u.t;
	int32_t sheduledTime =  currentTime;
	int tclockPerSoundRate = (SINCLAIR_FLAGS.T69888 / (882));
	if(getTapeState()&&(SINCLAIR_FLAGS.tapeSpeed==2))
	{
		tclockPerSoundRate*=3;
	}
    int old_size = aSOUND_EVENTS_size();
    int sound_size1 = sound_size();
    int rdelay = waitFor();
    int cntPush = 0;
    //mprintf("flag128=%d\r\n",flag128);
    while(aSOUND_EVENTS_size())
    {
    	int amp = 0;

    	if(currentTime>=sheduledTime)
    	{
    		union a_event next  = aSOUND_EVENTS[aSOUND_EVENTS_tail];
    		if(SINCLAIR_FLAGS.flag128)
    		{
    			//soundModule_sm.execute(nextEv);
    			execute(&next);
    		}

			if((next.u.ay==0)&&(next.u.data==0)||(next.u.data==1))
			{
				fuller = fuller_next;
				fuller_next = next.u.data?(MAX_VOLUME/64):(-MAX_VOLUME/64);
			}
			sheduledTime = next.u.t;
			aSOUND_EVENTS_pop();
    	}
    	else
    	{
			if(SINCLAIR_FLAGS.flag128)
			{
				//soundModule_sm.
				incrTick(tclockPerSoundRate);
				amp = (getSign())*MAX_VOLUME/2/0x10000;
			}
			cntPush++;
			//waitFor();
			//fuller =fuller*990/1024;
			push_pair(fuller+amp+MAX_VOLUME/2,fuller+amp+MAX_VOLUME/2);

			currentTime += tclockPerSoundRate;
    	}
    }
    while(currentTime<sheduledTime)
    {
    	int amp = 0;
    	if(SINCLAIR_FLAGS.flag128)
    	{
    		//soundModule_sm.

			incrTick(tclockPerSoundRate);
			amp = (getSign())*MAX_VOLUME/2/0x10000;
			//amp = (getSign()+0x8000)*1903/0x10000;
    	}
    	cntPush++;
    	//waitFor();
    		//fuller =fuller*990/1024;
		push_pair(fuller+amp+MAX_VOLUME/2,fuller+amp+MAX_VOLUME/2);
    	currentTime += tclockPerSoundRate;
    }
    int sound_size2 = sound_size();
   // mprintf("sound events_size %d delay %d cntPush = %d %d %d\r\n",old_size,rdelay,cntPush,sound_size1,sound_size2);
}

void out(u16 port, u8 value)
{
	if ((port&0xFF)==0x00FE)//перехват порта 0xFE
	{
		if(SINCLAIR_FLAGS.border != (value&0x7)||tstates <1000)
		{
			SINCLAIR_FLAGS.border = value&0x7;//D[0-2] border color
			int indB = (ILI9341_LCD_PIXEL_WIDTH+20)*(tstates)/SINCLAIR_FLAGS.T69888-10;
			if(indB<0)indB = 0;
			if(indB>ILI9341_LCD_PIXEL_WIDTH-1)indB = ILI9341_LCD_PIXEL_WIDTH-1;

			SINCLAIR_FLAGS.borderArray[indB] = SINCLAIR_FLAGS.border;
		}
		if(value&(0b11000)) //sound bit
		{
			if(soundBitState!=1)
			{
				soundBitState = 1;
				soundEvents(1);
			}
			//SOUND_GPIO_Port->BSRR = SOUND_Pin;
		}
		else
		{
			if(soundBitState!=0)
			{
				soundBitState = 0;
				soundEvents(0);
			}
			//SOUND_GPIO_Port->BSRR = (uint32_t)SOUND_Pin<<16U;
		}
	}
	//else if((port&0b10000010)==0)
	else if(port==0x7ffd)
	{
		setMemPages(value);
	}
	else if(port==0xfffd||port==0xfefd)
	{
		ay_write_command(value);
	}
	else if(port==0xbffd||port==0xbefd)
	{
		ay_write_data(value);
	}
}
void poke16(u16 addr, u16 value)
{
	poke(addr, value);
	poke(addr+1, value>>8);
}

u16 peek16(u16 addr)
{
	return ((peek(addr+1)<<8)|peek(addr));
}
struct CBuff
{
	u8 bb[0x40];
	int head;
	int tail;
}SB;

void SB_init()
{
	SB.head = 0;
	SB.tail    = 0;
}
void SB_put(u8 bt)
{
	SB.bb[SB.head] = bt;
	SB.head++;
	SB.head&=0x3f;
}
u8 SB_get()
{
	u8 res = SB.bb[SB.tail];
	SB.tail++;
	SB.tail&=0x3f;
	return res;
}
int SB_size()
{
	return (SB.head-SB.tail)&0x3f; 
}
FRESULT f_read_b(FIL* fp, void* buff, UINT btr, UINT* br)
{
	int k;
	if(SB_size()<btr)
	{
		UINT bytes = 0;
		u8 bbt[0x20];
		f_read(fp,bbt,0x20,&bytes);
		for(k=0;k<bytes;k++)
		{
			SB_put(bbt[k]);
		}
	}
        u8 *pnt =buff;
	int rb = 0;
	for(k=0;k<btr&&SB_size();k++)
	{
		pnt[k] = SB_get();
		rb++;
	}
	*br = rb;
	return FR_OK;
}
void poke_page(int page,u16 addr, u8 value)
{
	RAM[page][addr&0x3fff] = value; 
}

int freadFile_vers(char* fileName)
{
	FIL 		MyFile;
	if(f_open(&MyFile,(TCHAR*)fileName, FA_READ)==FR_OK)
	{
	clearFullScreen();
	SB_init();
	uint8_t bu8=0;
	uint16_t bu16=0;
	int version = 1;
	int isCompressed = 0;
	MEMPTR = 0;
	tstates = 11200;
	interrupts_enabled_at = 0;
	UINT BytesRead;
	f_read_b(&MyFile,&bu8,1,&BytesRead); A = bu8;
	f_read_b(&MyFile,&bu8,1,&BytesRead); F = bu8;
	f_read_b(&MyFile,&BC,2,&BytesRead);
	f_read_b(&MyFile,&HL,2,&BytesRead);
	f_read_b(&MyFile,&PC,2,&BytesRead);
	if(PC==0)
	{
		version = 2;
	}
	f_read_b(&MyFile,&SP,2,&BytesRead);
	f_read_b(&MyFile,&bu8,1,&BytesRead); I = bu8;
	f_read_b(&MyFile,&bu8,1,&BytesRead); R = bu8;
	f_read_b(&MyFile,&bu8,1,&BytesRead); R|=(bu8&1)<<7;
	R7=R;
	if(bu8&(1<<5))
	{
		isCompressed = 1;
	}
	f_read_b(&MyFile,&DE,2,&BytesRead);
	f_read_b(&MyFile,&BC_,2,&BytesRead);
	f_read_b(&MyFile,&DE_,2,&BytesRead);
	f_read_b(&MyFile,&HL_,2,&BytesRead);
	f_read_b(&MyFile,&bu8,1,&BytesRead); A_ = bu8;
	f_read_b(&MyFile,&bu8,1,&BytesRead); F_ = bu8;
	f_read_b(&MyFile,&IY,2,&BytesRead);
	f_read_b(&MyFile,&IX,2,&BytesRead);
	f_read_b(&MyFile,&bu8,1,&BytesRead); IFF1 = bu8;
	f_read_b(&MyFile,&bu8,1,&BytesRead); IFF2 = bu8;
	f_read_b(&MyFile,&bu8,1,&BytesRead); IM   = bu8&3;
	halt = 0;
	if(version>1)
	{
		int b128 = 0;
		f_read_b(&MyFile,&bu16,2,&BytesRead);
		mprintf("Add Length=%d\r\n",bu16);
		if(bu16 == 23)
		{
			version = 2;
		}
		else if(bu16 == 54)
		{
			version = 3;
		}
		else if(bu16 == 55)
		{
			version = 4;
		}

		f_read_b(&MyFile,&bu16,2,&BytesRead); PC = bu16;
		f_read_b(&MyFile,&bu8,1,&BytesRead);
		if(bu8>1)
		{
			//?128
			b128 = 1;
			SINCLAIR_FLAGS.flag128 = 1;
			initRam();
		}
		else
		{
			b128 = 0;
			SINCLAIR_FLAGS.flag128 = 0;
			initRam();
		}
		f_read_b(&MyFile,&bu8,1,&BytesRead); mprintf("35=%d\r\n",bu8);
		if(b128)
		{
			setMemPages(bu8);
		}
		f_read_b(&MyFile,&bu8,1,&BytesRead); mprintf("36=%d\r\n",bu8);
		f_read_b(&MyFile,&bu8,1,&BytesRead); mprintf("37=%d\r\n",bu8);
		f_read_b(&MyFile,&bu8,1,&BytesRead); mprintf("38=%d\r\n",bu8);
		char sound[0x10];
		f_read_b(&MyFile,sound,0x10,&BytesRead);
		if(version>=3)
		{
			f_read_b(&MyFile,&bu16,2,&BytesRead);  mprintf("LOW_T %x\r\n",bu16);
			f_read_b(&MyFile,&bu8,1,&BytesRead);  mprintf("HIGH_T %x\r\n",bu8);
			f_read_b(&MyFile,&bu8,1,&BytesRead);  mprintf("58 %x\r\n",bu8);
			f_read_b(&MyFile,&bu8,1,&BytesRead);  mprintf("59 %x\r\n",bu8);
			f_read_b(&MyFile,&bu8,1,&BytesRead);  mprintf("60 %x\r\n",bu8);
			f_read_b(&MyFile,&bu8,1,&BytesRead);  mprintf("61 %x\r\n",bu8);
			f_read_b(&MyFile,&bu8,1,&BytesRead);  mprintf("62 %x\r\n",bu8);
			f_read_b(&MyFile,sound,10,&BytesRead); mprintf("63 keyb[10]=\r\n");
			f_read_b(&MyFile,sound,10,&BytesRead); mprintf("73 ascii[10]=\r\n");
			
			f_read_b(&MyFile,&bu8,1,&BytesRead);  mprintf("83 %x\r\n",bu8);
			f_read_b(&MyFile,&bu8,1,&BytesRead);  mprintf("84 %x\r\n",bu8);
			f_read_b(&MyFile,&bu8,1,&BytesRead);  mprintf("85 %x\r\n",bu8);
			
			if(version==4)
			{
				f_read_b(&MyFile,&bu8,1,&BytesRead);  mprintf("86 %x\r\n",bu8);
			}
			
		}
		for(int p=0;p<8;p++)
		{
			f_read_b(&MyFile,&bu16,2,&BytesRead);mprintf("size= %x\r\n",bu16);
			f_read_b(&MyFile,&bu8,1,&BytesRead);  mprintf("page %x\r\n",bu8);
			int addr = 0x0000;
			int page = -1;
			mprintf("b128 %x\r\n",b128);
			if(!SINCLAIR_FLAGS.flag128)
			{
				b128 = 0;
			}
			if(!b128)
			{
				
				addr = 0;
				if(bu8==4)
				{
					addr = 0x8000;
					page = 2;
				}
				else if(bu8==5)
				{
					addr = 0xc000;
					page = 0;
				}
				else if(bu8==8)
				{
					addr = 0x4000;
					page = 5;
				}
				else page = -1;
			}
			else
			{
				page =  bu8-3;
				addr  = 0;
				if(bu8==3)
				{
					addr = 0xc000;
				}
				else if(bu8==5)
				{ 
					addr = 0x8000;
				}
				else if(bu8==8)
				{
					addr = 0x4000;
				}
				
			}
			if(page>=0&&page<=7)
			{
				if(bu16==0xffff)
				{
					if(page>=0)
					{
						for(int k=0;k<0x4000;k++)
						{
							f_read_b(&MyFile,&bu8,1,&BytesRead); 
							poke_page(page,addr,bu8);addr++;
						}
					}
				}
				else
				{
					if(page>=0)
					{
						int bytes = 0;
						uint8_t rl;
						while(bytes<bu16)
						{
							f_read_b(&MyFile,&bu8,1,&BytesRead);  bytes++;
							if(bu8==0xed)
							{
								f_read_b(&MyFile,&bu8,1,&BytesRead);  bytes++;
								if(bu8==0xed)
								{
									//ifs.read((char*)&rl,1); 
									//ifs.read((char*)&bu8,1); bytes++;
									f_read_b(&MyFile,&rl,1,&BytesRead);bytes++;
									f_read_b(&MyFile,&bu8,1,&BytesRead);  bytes++;
									for(int k=0;k<rl;k++)
									{
										poke_page(page,addr,bu8);addr++;
									}
								}
								else
								{
									poke_page(page,addr,0xed);
									addr++;
									poke_page(page,addr,bu8);
									addr++;
								}
							}
							else 
							{
								poke_page(page,addr,bu8);
								addr++;
							}
						}
					}
				}
			}
		}
	}
	else
	{
		SINCLAIR_FLAGS.flag128 = 0;
		initRam();
		int addr = 0x4000;
		uint8_t rl;
		if(!isCompressed)
		{
			//read 48K raw
			for(int k=0;k<0x10000-0x4000;k++)
			{
				//ifs.read((char*)&bu8,1); 
				f_read_b(&MyFile,&bu8,1,&BytesRead); 
				poke(addr,bu8);addr++;
			}
		}
		else
		{
			while(addr<0x10000)
			{
				//ifs.read((char*)&bu8,1); 
				f_read_b(&MyFile,&bu8,1,&BytesRead); 
				if(BytesRead)
				{
					if(bu8==0xed)
					{
						//ifs.read((char*)&bu8,1); 
						f_read_b(&MyFile,&bu8,1,&BytesRead); 
						if(bu8==0xed)
						{
							f_read_b(&MyFile,&rl,1,&BytesRead); 
							f_read_b(&MyFile,&bu8,1,&BytesRead); 
			//				ifs.read((char*)&rl,1); 
			//				ifs.read((char*)&bu8,1); 
							for(int k=0;k<rl;k++)
							{
								poke(addr,bu8);addr++;
							}
						}
						else
						{
							poke(addr,0xed);
							addr++;
							poke(addr,bu8);
							addr++;
						}
					}
					else 
					{
						poke(addr,bu8);
						addr++;
					}
				}
				else
				{
					mprintf("addr %x\r\n",addr);
					addr = 0x10000;
				}
					
			}
		}
		//PRINT(version);
		//PRINT(isCompressed);
		return 1;
	}
	
	f_close(&MyFile);
	//f_mount(NULL,USERPath, 0);
	//FATFS_UnLinkDriver(USERPath);
	return 1;
	}
	else
	{
		return 0;
	}


}



void clearFullScreen()
{
	LCD_fillRect(0,0,LCD_getWidth(),LCD_getHeight(),BLACK);
}

int  menuLines = 0;

// child nodes
char* BRIGHTNESS = "BRIGHT";
char* TAPE = "TAPE %d"; //0 dis


char* EXIT 		 = "Exit to Menu";

int colorTable[2][3] =
{
		{LGRAY,DGRAY,BLACK},
		{GREEN,YELLOW,BLACK}
};
int z48_z128_dialog_dispatch(struct SYS_EVENT* ev)
{
	if(ev->message)
	{
		mprintf("z48_z128_dialog_dispatch  = %x\r\n",ev->message);
	}
	if(ev->message==MESS_OPEN)
	{
		cleanAudioBuffer();
		ev->message = MESS_REPAINT;
		return 0;
	}
	else if(ev->message==MESS_REPAINT)
	{
		const uint16_t yScr = (240-192)/2-4;
		const uint16_t xScr = (320)/2;

		LCD_Draw_Text(JS_LINES[SINCLAIR_FLAGS.joystickMode],xScr-strlen(JS_LINES[SINCLAIR_FLAGS.joystickMode])*8,yScr+16, BLACK, 2,menuLines==0?colorTable[SINCLAIR_FLAGS.bColor][0]:colorTable[SINCLAIR_FLAGS.bColor][1]);
		LCD_Draw_Text(BRIGHTNESS,xScr-strlen(BRIGHTNESS)*8-64,yScr+16+32, BLACK, 2,menuLines==1?colorTable[SINCLAIR_FLAGS.bColor][0]:colorTable[SINCLAIR_FLAGS.bColor][1]);
		char sBuff[0x10];
		sprintf(sBuff,TAPE,SINCLAIR_FLAGS.tapeSpeed);
		LCD_Draw_Text(sBuff,xScr-strlen(BRIGHTNESS)*8+64,yScr+16+32, BLACK, 2,menuLines==2?colorTable[SINCLAIR_FLAGS.bColor][0]:colorTable[SINCLAIR_FLAGS.bColor][1]);
		LCD_Draw_Text(EXIT,xScr-strlen(EXIT)*8,yScr+16+64, BLACK, 2,menuLines==3?colorTable[SINCLAIR_FLAGS.bColor][0]:colorTable[SINCLAIR_FLAGS.bColor][1]);
		char* Map =joyMap[SINCLAIR_FLAGS.joystickMode];
		LCD_Draw_Char(Map[0],xScr-8,yScr+16+96,BLACK,2,colorTable[SINCLAIR_FLAGS.bColor][1]);
		LCD_Draw_Char(Map[1],xScr-8,yScr+16+96+64,BLACK,2,colorTable[SINCLAIR_FLAGS.bColor][1]);
		LCD_Draw_Char(Map[2],xScr-8-48,yScr+16+96+32,BLACK,2,colorTable[SINCLAIR_FLAGS.bColor][1]);
		LCD_Draw_Char(Map[3],xScr-8+48,yScr+16+96+32,BLACK,2,colorTable[SINCLAIR_FLAGS.bColor][1]);
		LCD_Draw_Char(Map[4],xScr-8-96,yScr+16+96+64,BLACK,2,colorTable[SINCLAIR_FLAGS.bColor][1]);
		LCD_Draw_Char(Map[5],xScr-8-48,yScr+16+96+64,BLACK,2,colorTable[SINCLAIR_FLAGS.bColor][1]);
		HAL_Delay(300);
		clearKey();
		ev->message = MESS_IDLE;
		return 0;
	}
	else if(ev->message==MESS_IDLE)
	{

	}
	else if(ev->message==MESS_KEYBOARD)
	{
		if(ev->param1==K_ESC)
		{
			setAttr();
			HAL_Delay(200);
			clearKey();
			ev->message = MESS_CLOSE;
			return 0;
		}
		else if(ev->param1==K_FIRE)
		{
			if(menuLines==3)
			{
				tape_close();
				cleanAudioBuffer();
				ev->message = MESS_CLOSE;
				popMenuFunc();
				return 0;
			}
			else if(menuLines==1)
			{
				SINCLAIR_FLAGS.bColor = !SINCLAIR_FLAGS.bColor;
				ev->message = MESS_REPAINT;
				HAL_Delay(200);
				clearKey();
				return 1;
			}
			else
			{
				setAttr();
				HAL_Delay(200);
				clearKey();
				ev->message = MESS_CLOSE;
				return 0;
			}
		}
		else if(ev->param1==K_UP)
		{
			menuLines =( menuLines+3)%4;
		}
		else if(ev->param1==K_DOWN)
		{
			menuLines =( menuLines+1)%4;
		}
		else if(ev->param1==K_RIGHT)
		{
			if(menuLines==0)
			{
				SINCLAIR_FLAGS.joystickMode = (SINCLAIR_FLAGS.joystickMode+joyMapSize-1)%joyMapSize;
			}
			else if(menuLines==1)
			{
				setBacklight(getBacklight()+5);
			}
			else if(menuLines==2)
			{
				SINCLAIR_FLAGS.tapeSpeed =(SINCLAIR_FLAGS.tapeSpeed+1)%3;
				tapeSetTime();
			}
			else if(menuLines==3)
			{

			}
		}
		else  if(ev->param1==K_LEFT)
		{
			if(menuLines==0)
			{
				SINCLAIR_FLAGS.joystickMode = (SINCLAIR_FLAGS.joystickMode+1)%joyMapSize;
			}
			else if(menuLines==1)
			{
				setBacklight(getBacklight()-5);
			}
			else if(menuLines==2)
			{
				SINCLAIR_FLAGS.tapeSpeed =(SINCLAIR_FLAGS.tapeSpeed+2)%3;
				tapeSetTime();
			}
			else if(menuLines==3)
			{

			}
		}
		ev->message = MESS_REPAINT;
		return 1;
	}
	ev->message=MESS_IDLE;
	return 1;
}

int z48_z128_dispatch(struct SYS_EVENT* ev)
{
	//if(ev->message)
	{
		//mprintf("z48_z128_dispatch  = %x\r\n",ev->message);
	}
	if(ev->message==MESS_OPEN)
	{
		if(getModeFile()==0)
		{
			SINCLAIR_FLAGS.flag128 = 0;
		}
		else
		{
			DirtyMemorySet();
			SINCLAIR_FLAGS.flag128 = 1;
		}
		initRam();
		if(getFileType(getFileName())==T_TAP)
		{
			TapeState_constructor();
			if(tape_open(getFileName()))
			{
				clearFullScreen();
				initRam();
				z80_reset(1);
				KEY_QUE_init();
				mprintf("KEY_QUE_size()=%d\r\n",KEY_QUE_size());
				int TM_OUT= 48;
				if(SINCLAIR_FLAGS.flag128)
				{
					SINCLAIR_FLAGS.tapeSpeed = 1;

					KEY_QUE_put(ENTER,0,TM_OUT*4);

					KEY_QUE_put(ENTER,1,TM_OUT);
					KEY_QUE_put(ENTER,0,TM_OUT);
				}
				else
				{
					SINCLAIR_FLAGS.tapeSpeed = 1;

					KEY_QUE_put(KEY_J,0,TM_OUT*4);

					KEY_QUE_put(KEY_J,1,TM_OUT);
					KEY_QUE_put(KEY_J,0,TM_OUT);
					KEY_QUE_put(SYMB_SHIFT,1,TM_OUT);
					KEY_QUE_put(KEY_P,1,TM_OUT);
					KEY_QUE_put(KEY_P,0,TM_OUT);
					KEY_QUE_put(KEY_P,1,TM_OUT);
					KEY_QUE_put(KEY_P,0,TM_OUT);
					KEY_QUE_put(SYMB_SHIFT,0,TM_OUT);
					KEY_QUE_put(ENTER,1,TM_OUT);
					KEY_QUE_put(ENTER,0,TM_OUT);
				}

				mprintf("KEY_QUE_size()=%d\r\n",KEY_QUE_size());
				//KEY_QUE_put(((14-8)<<8)|0);
				tapeStart();
				tapeSetTime();
				ev->message = MESS_REPAINT;
				return 1;
			}
			else
			{
				ev->message = MESS_CLOSE;
				return 1;
			}
		}
		else if(getFileType(getFileName())==T_Z80)
		{
			initRam();
			ay_reset();
			KEY_QUE_init();
			z80_reset(1);
			DirtyMemorySet();
			mprintf("full name%s\r\n",getFileName());
			if(!freadFile_vers(getFileName()))
			{
				ev->message = MESS_CLOSE;
				return 0;
			}
			else
			{
				setAttr();
				ev->message = MESS_REPAINT;
				return 1;
			}

		}
		ev->message=MESS_IDLE;
		return 1;
	}
	else if(ev->message==MESS_IDLE||ev->message==MESS_KEYBOARD)
	{
		if(ev->message==MESS_KEYBOARD)
		{
			if(ev->param1==K_ESC)
			{
				setCurrentFunc(z48_z128_dialog_dispatch);
//				tape_close();
//				cleanAudioBuffer();
//				ev->message = MESS_CLOSE;
				ev->message = MESS_OPEN;
				return 0;
			}
		}
		int32_t tickstart = HAL_GetTick();
		soundEvents(100);
		{
			//int tstop = tstates+T69888;
			//for(;tstates<T69888;)
			{
				//z80_interrupt();
				z80_run(SINCLAIR_FLAGS.T69888);
			}
		}
		//mprintf("tt=%d\r\n",(HAL_GetTick() - tickstart));


		proccesSoundEvents_time();


		tstates -=SINCLAIR_FLAGS.T69888;
		interrupts_enabled_at -=SINCLAIR_FLAGS.T69888;
		SINCLAIR_FLAGS.frameCounter++;

		ev->message=MESS_REPAINT;
		return 1;
	}
	else if(ev->message==MESS_REPAINT)
	{
		uint32_t tickperv = HAL_GetTick();

//		static uint16_t old_border =0xffff;
		//~ HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
		//~ if(old_border!=(border&0x7))
		//~ {
			//~ old_border = border&0x7;
			//~ const uint16_t yScr = (240-192)/2;
			//~ const uint16_t xScr = (320-256)/2;
			//~ uint8_t buff[3];
			//~ buff[0] = 0xd7*(!!(border&2));
			//~ buff[1] = 0xd7*(!!(border&4));
			//~ buff[2] = 0xd7*(!!(border&1));
			//~ LCD_FullRect3(xScr-4,yScr-4,buff,4,192+8) ;
			//~ LCD_FullRect3(xScr+256,yScr-4,buff,4,192+8) ;
			//~ LCD_FullRect3(xScr-4,yScr-4,buff,256+8,4) ;
			//~ LCD_FullRect3(xScr-4,yScr+192,buff,256+8,4) ;
		//~ }

		// x tile_ofs = x (0,32)
		// y tile_ofs
		int yb,xb;
		for( yb=0;yb<24;yb++)
		{
			for(xb=0;xb<32;xb++)
			{
				int tileAddr = xb+yb*32;
				if(ATTR_RAM_MOD[tileAddr>>3] & (1<<(tileAddr&0x3)))
				{
					//~ ATTR_RAM_MOD[tileAddr>>3] &= 0xff-(1<<(tileAddr&0x3));
					uint8_t attr = ATTR_RAM[tileAddr];
					uint8_t Br = !!(attr&64);
					uint8_t scale = (Br?0xff:0xd7);

					if((attr&128)&&((tickperv/1000)&1))
					{
						scale = (Br?0xd7:0xff);
					}
					uint8_t RInc= scale*!!(attr&2);
					uint8_t GInc= scale*!!(attr&4);
					uint8_t BInc= scale*!!(attr&1);

					uint8_t Rpap= scale*!!(attr&16);
					uint8_t Gpap= scale*!!(attr&32);
					uint8_t Bpap= scale*!!(attr&8);
					//~ ATTR_RAM_MOD[tileAddr>>3] &= ~(1<<(tileAddr&0x3));
#ifdef COLOR_3BYTES
					uint8_t buff[8][24];
					int yt;
					int xScr = xb*8+(320-256)/2;
					int yScr = yb*8+(240-192)/2;
					for(yt=0;yt<8;yt++)
					{
						int y = yt+yb*8;
						//~ uint8_t lineColor[32];
						uint16_t vr = ((y<<8)&0x0700)|((y<<2)&0xe0)|((y<<5)&0x1800);
						uint8_t   val  = VIDEO_RAM[vr+xb];
						int bit;
						for(bit=0;bit<8;bit++)
						{


							int bt          =((val>>(7-bit))&1);
							buff[yt][bit*3+0] = bt?(RInc):(Rpap);
							buff[yt][bit*3+1] = bt?(GInc):(Gpap);
							buff[yt][bit*3+2] = bt?(BInc):(Bpap);
						}
						//~ LCD_Write8line(xScr,yScr,buff) ;
					}
					LCD_Write8x8line(xScr,yScr,&buff[0][0]) ;
#else
					uint16_t buff[8][8];
					int yt;
					int xScr = xb*8+(320-256)/2;
					int yScr = yb*8+(240-192)/2;
					for(yt=0;yt<8;yt++)
					{
						int y = yt+yb*8;
						//~ uint8_t lineColor[32];
						uint16_t vr = ((y<<8)&0x0700)|((y<<2)&0xe0)|((y<<5)&0x1800);
						uint8_t   val  = VIDEO_RAM[vr+xb];
						int bit;
						if(SINCLAIR_FLAGS.bColor)
						{
							for(bit=0;bit<8;bit++)
							{
								int bt          =((val>>(7-bit))&1);
								int Rc = (bt?(RInc):(Rpap));
								int Gc = (bt?(GInc):(Gpap));
								int Bc = (bt?(BInc):(Bpap));
								buff[yt][bit] = ((Rc>>3)<<11)|((Gc>>2)<<5)|(Bc>>3);
							}
						}
						else
						{
							for(bit=0;bit<8;bit++)
							{
								int bt          =((val>>(7-bit))&1);
								int Rc = (bt?(RInc):(Rpap));
								int Gc = (bt?(GInc):(Gpap));
								int Bc = (bt?(BInc):(Bpap));
								int FC = (Rc+Gc+Bc)/3;
								buff[yt][bit] = ((FC>>3)<<11)|((FC>>2)<<5)|(FC>>3);
							}
						}
						//~ LCD_Write8line(xScr,yScr,buff) ;

					}
					LCD_Write8x8line16(xScr,yScr,&buff[0][0]) ;
#endif

				}
			}
		
			
		}
		const uint16_t yScr = (240-192)/2;
		const uint16_t xScr = (320-256)/2;
		for(int line = 0;line< ILI9341_LCD_PIXEL_WIDTH;line++)
		{
			if(SINCLAIR_FLAGS.borderArray[line]!=0xff)
			{
				SINCLAIR_FLAGS.border = SINCLAIR_FLAGS.borderArray[line];
			}
			SINCLAIR_FLAGS.borderArray[line] = 0xff;
			uint8_t buff[3];
			buff[0] = 0xd7*(!!(SINCLAIR_FLAGS.border&2));
			buff[1] = 0xd7*(!!(SINCLAIR_FLAGS.border&4));
			buff[2] = 0xd7*(!!(SINCLAIR_FLAGS.border&1));
			if(!SINCLAIR_FLAGS.bColor)
			{
				int FC = (buff[0]+buff[1]+buff[2])/3;
				buff[0] = FC;
				buff[1] = FC;
				buff[2] = FC;
			}

			if(line<yScr)
			{
				LCD_FullRect3(0,line,buff,ILI9341_LCD_PIXEL_HEIGHT,1);
			}
			else if(line>=yScr+192)
			{
				LCD_FullRect3(0,line,buff,ILI9341_LCD_PIXEL_HEIGHT,1);
			}
			else
			{
				LCD_FullRect3(0,line,buff,xScr,1);
			    LCD_FullRect3(xScr+256,line,buff,xScr,1);
			}


		}
		
		clearAttr();
		ev->message=MESS_IDLE;
		screen_IRQ = 1;
		return 1;
	}
	ev->message=MESS_IDLE;
	return 0;
}


