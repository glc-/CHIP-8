#include <math.h>   /* pow() */
#include <stdio.h>
#include <stdlib.h> /* rand() */
#include <string.h> /* memset() */
#include <time.h>
#include <SDL.h>

#define W 64
#define H 32
#define max(A, B)  ((A) > (B) ? (A) : (B))

struct cpu {
  unsigned char mem[0x1000], V[16], DT, ST, gfx[W*H/8], keys[16], wait_key;
  unsigned short stack[16], I, PC, SP;
};

unsigned fonts[16] = { 0xF999F, 0x26227, 0xF1F8F, 0xF1F1F,
		       0x99F11, 0xF8F1F, 0xF8F9F, 0xF1244,
		       0xF9F9F, 0xF9F1F, 0xF9F99, 0xE9E9E,
		       0xF888F, 0xE999E, 0xF8F8F, 0xF8F88 
};

void font_init (struct cpu* cpu)
{
  for (int i=0,k=0; i<16; i++)
    for (int j=16; j>=0; j-=4)
      cpu->mem[k++] = ( (fonts[i]>>j)&0xF )<<4;
}
			
void load (const char* n, struct cpu* cpu)
{
  FILE *f = fopen(n, "r");
  fread( &(cpu->mem[0x200]), sizeof(char), 0xe00, f);
  fclose(f);
}

void cpu_init  (struct cpu* cpu)
{
  memset(cpu->mem,0,0x1000);
  memset(cpu->gfx,0,W*H/8);
  memset(cpu->V,0,16);
  memset(cpu->stack,0,16);
  memset(cpu->keys,0,16);
  cpu->PC = 0x200;
  cpu->I = 0;
  cpu->SP = 0;
  cpu->DT = 0;
  cpu->ST = 0;
  cpu->wait_key = 0;
  font_init(cpu);
}

#define p struct cpu* cpu, unsigned short op, unsigned short nnn, \
unsigned short n, unsigned short x, unsigned short y, unsigned short kk
#define PC_ cpu->PC
#define SP cpu->SP
#define I cpu->I
#define DT cpu->DT
#define ST cpu->ST
#define mem cpu->mem
#define V cpu->V
#define stack cpu->stack
#define VF V[0xF]

/*------------------------------------------------------------------------,
| nnn or addr - A 12-bit value, the lowest 12 bits of the instruction     |
| n or nibble - A 4-bit value, the lowest 4 bits of the instruction       |
| x - A 4-bit value, the lower 4 bits of the high byte of the instruction |
| y - A 4-bit value, the upper 4 bits of the low byte of the instruction  |
| kk or byte - An 8-bit value, the lowest 8 bits of the instruction       |
'------------------------------------------------------------------------*/
void nop       (p) { /* Not part of Chip8 ISA */                               }
void add_vx_kk (p) { V[x] += kk;                                               }
void and       (p) { V[x] &= V[y];                                             }
void or        (p) { V[x] |= V[y];                                             }
void xor       (p) { V[x] ^= V[y];                                             }
void rnd       (p) { V[x]  = (rand() % 255) & kk;                              }
void ld_vx_vy  (p) { V[x]  = V[y];                                             }
void ld_vx_kk  (p) { V[x]  = kk;                                               }
void ld_vx_dt  (p) { V[x]  = DT;                                               }
void subb      (p) { VF    = ( V[x] > V[y] ) ? 1 : 0;    V[x] -= V[y];         }
void subnb     (p) { VF    = ( V[y] > V[x] ) ? 1 : 0; V[x] = V[y] - V[x];      }
void shl       (p) { VF    = V[x >> 3]; V[x] <<= 1;                            }
void shr       (p) { VF    = ( V[x] & 1 );    V[x] >>= 1;                      }
void add_i_vx  (p) { I    += V[x];                                             }
void ld_I_nnn  (p) { I     = nnn;                                              }
void ld_f_vx   (p) { I     = V[x] * 5;  /* 5 bytes offset. */                  }
void ret       (p) { PC_   = stack[SP--];                                      }
void jp_nnn    (p) { PC_   = nnn;                                              }
void jp_v0_nnn (p) { PC_   = nnn + V[0];                                       }
void ld_dt_vx  (p) { DT    = V[x];                                             }
void ld_st_vx  (p) { ST    = V[x];                                             }
void addc      (p) { n = V[x]+V[y]; VF = (n>0xFF) ? 1:0; V[x] = n&0xFF;        }
void call_nnn  (p) { stack[++SP] = PC_;          PC_ = nnn;                    }
void ld_vx_k   (p) { cpu->wait_key = x | 0x80;                                 }
void se_vx_kk  (p) { if ( V[x] == kk   )         PC_ += 2;                     }
void sne_vx_kk (p) { if ( V[x] != kk   )         PC_ += 2;                     }
void se_vx_vy  (p) { if ( V[x] == V[y] )         PC_ += 2;                     }
void sne_vx_vy (p) { if ( V[x] != V[y] )         PC_ += 2;                     }
void skp_vx    (p) { if (   cpu->keys[ V[x] ]  ) PC_ += 2;                     }
void sknp_vx   (p) { if (! (cpu->keys[ V[x] ]) ) PC_ += 2;                     }
void cls       (p) { for (int i=0; i< W*H/8; i++) { cpu->gfx[i] = 0;    }      }
void wreg      (p) { for (int i=0; i<=x; i++)     { mem[I + i]  = V[i]; }      }
void rreg      (p) { for (int i=0; i<=x; i++)     { V[i] = mem[I + i];  }      }
void bcd       (p) { mem[I]=V[x]/100; mem[I+1]=(V[x]%100)/10; mem[I+2]=V[x]%10;}
void drw       (p) { VF = 0;    unsigned a, b, r, s;
                     for (int i=0; i<n; i++)
		       for (int j=0; j<W/8; j++) { 
			 r = (V[x] + j);    s = (V[y] + i);
			 a = cpu->gfx[s*8 + r/8] >> (7-r%8) &1;
			 b = mem[(I+i)&0xFFF] >> (7-j) &1;
			 cpu->gfx[s*8 + r/8] ^= b*(unsigned) pow(2, 7-r%8);
			 VF = (a & b) ? 1 : VF;
		       }                                                       }

void (*chip8_table[16]) (p);
void (*zero_table[3])   (p);
void (*alu_table[9])    (p);
void (*e_table[2])      (p);
void (*ft[9])           (p); 
void zero               (p);
void alu                (p);
void e                  (p);
void f                  (p);

void exec (struct cpu* cpu)
{
  if(PC_ > 0xFFE) return;
  unsigned short ins = (mem[PC_] << 8) | (mem[PC_+1]);    PC_ += 2;
  unsigned short op  = (ins >>12)&0xF;
  unsigned short nnn = (ins >> 0) & 0xFFF;
  unsigned short n   = (ins >> 0) & 0x00F;
  unsigned short x   = (ins >> 8) & 0x00F;
  unsigned short y   = (ins >> 4) & 0x00F;
  unsigned short kk  = (ins >> 0) & 0x0FF;
  chip8_table[op] (cpu, op, nnn, n, x, y, kk);
}

#undef p
#undef PC_
#undef SP
#undef I
#undef DT
#undef ST
#undef mem
#undef V
#undef stack

int main (int argc, char** argv)
{

  struct cpu* cpu = (struct cpu*) malloc(sizeof(struct cpu)); cpu_init(cpu);

  unsigned ins_per_frame=10, max_consecutive_ins=0, frames_done=0, running=1;
  struct timespec start_t; clock_gettime(CLOCK_MONOTONIC_RAW, &start_t);

  /* Create a screen. */
  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window *window = SDL_CreateWindow(argv[1], SDL_WINDOWPOS_CENTERED, \
                                        SDL_WINDOWPOS_CENTERED, 512, 256, NULL);
  SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, \
					      SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);

  if (argc<2) {
    ins_per_frame = 2;
    unsigned char no_rom_fonts[20] = { 0xC2, 0xA2, 0x92, 0x8A, 0x86, 0x38, 0x44,
				       0xC6, 0x44, 0x38, 0xFC, 0x84, 0xFC, 0x88, 
				       0x84, 0xC6, 0xAA, 0x92, 0x82, 0x82 };
    for (int i=0,k=0x100; i<20; i++)
      cpu->mem[k++]=no_rom_fonts[i];
    load("no_rom.bin", cpu);
  }
  else {
    load(argv[1], cpu);
  }
  
  while(running)
    {
      if(cpu->PC > 0xFFE) { running=0; break; }
      for(unsigned a=0; a<max_consecutive_ins && !(cpu->wait_key & 0x80); ++a)
        exec(cpu);

      for(SDL_Event ev; SDL_PollEvent(&ev); )
        switch(ev.type) {
	  unsigned k;
	case SDL_QUIT: running = 0; break;
	case SDL_KEYDOWN:
	case SDL_KEYUP:
	  switch (ev.key.keysym.sym) {
	  case SDLK_1: k = cpu->keys[0x1] = ev.type==SDL_KEYDOWN; break;
	  case SDLK_2: k = cpu->keys[0x2] = ev.type==SDL_KEYDOWN; break;
	  case SDLK_3: k = cpu->keys[0x3] = ev.type==SDL_KEYDOWN; break;
	  case SDLK_4: k = cpu->keys[0xC] = ev.type==SDL_KEYDOWN; break;
	  case SDLK_q: k = cpu->keys[0x4] = ev.type==SDL_KEYDOWN; break;
	  case SDLK_w: k = cpu->keys[0x5] = ev.type==SDL_KEYDOWN; break;
	  case SDLK_e: k = cpu->keys[0x6] = ev.type==SDL_KEYDOWN; break;
	  case SDLK_r: k = cpu->keys[0xD] = ev.type==SDL_KEYDOWN; break;
	  case SDLK_a: k = cpu->keys[0x7] = ev.type==SDL_KEYDOWN; break;
	  case SDLK_s: k = cpu->keys[0x8] = ev.type==SDL_KEYDOWN; break;
	  case SDLK_d: k = cpu->keys[0x9] = ev.type==SDL_KEYDOWN; break;
	  case SDLK_f: k = cpu->keys[0xE] = ev.type==SDL_KEYDOWN; break;
	  case SDLK_z: k = cpu->keys[0xA] = ev.type==SDL_KEYDOWN; break;
	  case SDLK_x: k = cpu->keys[0x0] = ev.type==SDL_KEYDOWN; break;
	  case SDLK_c: k = cpu->keys[0xB] = ev.type==SDL_KEYDOWN; break;
	  case SDLK_v: k = cpu->keys[0xF] = ev.type==SDL_KEYDOWN; break;

	  case SDLK_ESCAPE: running=0; break;
	  case SDLK_F1:  { 
	    ins_per_frame+=5;
	    printf("Ins / Frame = %d\n", ins_per_frame); } break; 
	  case SDLK_F2: { 
	    if (ins_per_frame>=10) ins_per_frame-=5; 
	    printf("Ins / Frame = %d\n", ins_per_frame); } break;
	  }
	  if(ev.type==SDL_KEYDOWN && (cpu->wait_key & 0x80))  {
	    cpu->wait_key        &= 0x7F;
	    cpu->V[cpu->wait_key] = k;
	  }
	}
      // Check how many frames we are _supposed_ to have rendered so far
      struct timespec cur_t;    clock_gettime(CLOCK_MONOTONIC_RAW, &cur_t);
      double elapsed = (( 1000.0*cur_t.tv_sec + 1e-6*cur_t.tv_nsec) - \
			(1000.0*start_t.tv_sec + 1e-6*start_t.tv_nsec))/1000;
      int frames = (int)(elapsed * 60) - frames_done;
      if(frames > 0)
        {
          frames_done += frames;
          // Update the timer registers
          if (cpu->DT > 0) { --cpu->DT; }
          if (cpu->ST > 0) { printf("Beep!\n\a"); --cpu->ST; }
          // Render graphics
          SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
          SDL_RenderClear(renderer);
          SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

          SDL_Rect* destRect = (SDL_Rect *)malloc(sizeof(SDL_Rect));
          destRect->x = 0;  destRect->y = 0;  destRect->w = 8;  destRect->h = 8;
          for(int i=0;i<H; i++)
            for(int j=0;j<W/8;j++)
              for(int k=7;k>=0;k--)
                if((cpu->gfx[(i*8)+j]>>k)&1==1) {
                  destRect->x = ((j*8) + (7-k)) *8;
                  destRect->y = i*8;
                  SDL_RenderFillRect(renderer, destRect);
                }
          free(destRect);
          SDL_RenderPresent(renderer);
        }
      // Adjust the instruction count to compensate for our rendering speed
      max_consecutive_ins = max(frames, 1) * ins_per_frame;
      // If CPU is waiting for a key, or if we no frame yet, consume time
      if((cpu->wait_key & 0x80) || !frames)
	SDL_Delay(1000/60);
    }
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  free(cpu);
  return 0;
}
#define p struct cpu* cpu, unsigned short op, unsigned short nnn, \
unsigned short n, unsigned short x, unsigned short y, unsigned short kk
#define PC_ cpu->PC
#define SP cpu->SP
#define I cpu->I
#define DT cpu->DT
#define ST cpu->ST
#define mem cpu->mem
#define V cpu->V
#define stack cpu->stack
#define VF V[0xF]

void (*chip8_table[16]) (p) = {     zero,     jp_nnn,   call_nnn,   se_vx_kk,
			       sne_vx_kk,   se_vx_vy,   ld_vx_kk,  add_vx_kk,
				     alu,  sne_vx_vy,   ld_I_nnn,  jp_v0_nnn,
				     rnd,        drw,          e,          f  };
void (*zero_table[3])  (p) = {       nop,        cls,        ret              };
void (*alu_table[9])   (p) = {  ld_vx_vy,         or,        and,        xor,
				    addc,       subb,        shr,      subnb,
                                     shl                                      };
void (*e_table[2])     (p) = {    skp_vx,    sknp_vx                          };
void (*ft[9])          (p) = {  add_i_vx,    ld_vx_k,       rreg,   ld_dt_vx,
                                    wreg,    ld_f_vx,   ld_st_vx,   ld_vx_dt,
                                     bcd                                      };
void zero (p) { zero_table[y?(n?2:1):0] (cpu, op, nnn, n, x, y, kk);           }
void alu  (p) { alu_table[n==0xe?8:n] (cpu, op, nnn, n, x, y, kk);             }
void e    (p) { e_table[n==0xe?0:n] (cpu, op, nnn, n, x, y, kk);               }
void f    (p) { unsigned u=kk%9; 
ft[u==3||u==6?(u==3?(n==5?3:0):(n==8?6:8)):u](cpu, op, nnn, n, x, y, kk);      }

#undef p
#undef PC_
#undef SP
#undef I
#undef DT
#undef ST
#undef mem
#undef V
#undef stack


    /*
      0xC2A2928A86
      **----*-
      *-*---*-
      *--*--*-
      *---*-*-
      *----**-

      0x3844C64438
      --***---
      -*---*--
      **---**-
      -*---*--
      --***---

      0xFC84FC8884
      ******--
      *----*--
      ******--
      *---*---
      *----*--

      0xC6AA928282
      **---**-
      *-*-*-*-
      *--*--*-
      *-----*-
      *-----*-
    */
    
    /* X,Y positioning */
    /*cpu->V[0] = 0xC; cpu->V[1] =  0x8; cpu->V[2] = 0x10;
      cpu->V[3] = 0x20; cpu->V[4] = 0x28; cpu->V[5] = 0x30;*/

    /* Assigning draw with different X,Y pos to mem */
    /*
    for (int i=512, j=1; i<522; i++) {
      if(i%2==0) 
	cpu->mem[i] = 0xD0+(unsigned char)j++;
      else
	cpu->mem[i] = 0x05;
    }
    */

/*
    cpu->mem[0x200] = 0x60; cpu->mem[0x201] = 0x0C; // V[0] = 12
    cpu->mem[0x202] = 0x61; cpu->mem[0x203] = 0x08; // V[1] = 8
    cpu->mem[0x204] = 0x62; cpu->mem[0x205] = 0x10; // V[2] = 16
    cpu->mem[0x206] = 0x63; cpu->mem[0x207] = 0x20; // V[3] = 32
    cpu->mem[0x208] = 0x64; cpu->mem[0x209] = 0x28; // V[4] = 40
    cpu->mem[0x20A] = 0x65; cpu->mem[0x20B] = 0x30; // V[5] = 48
    cpu->mem[0xF00] = 0x00; cpu->mem[0xF01] = 0xEE; // Letters Call 0xD00, this is their return

    cpu->mem[0x210] = 0x22; cpu->mem[0x211] = 0x20; //SLOWDRW
    cpu->mem[0x212] = 0x2D; cpu->mem[0x213] = 0x00; //128 cycle pause
    cpu->mem[0x214] = 0x00; cpu->mem[0x215] = 0xE0; //CLS
    cpu->mem[0x216] = 0x22; cpu->mem[0x217] = 0x20; //SLOWDRW
    cpu->mem[0x218] = 0x22; cpu->mem[0x219] = 0x20; //SLOWDRW -> Erase
    cpu->mem[0x21A] = 0x2E; cpu->mem[0x21B] = 0x00; //64 cycle pause
    cpu->mem[0x21C] = 0x22; cpu->mem[0x21D] = 0x40; //FLASH5
    cpu->mem[0x21E] = 0x1F; cpu->mem[0x21F] = 0xFE; //JP to end: 0xFFE

// SLOWDRW: Draw letter, call 0x400, spin 128 cycles, return at 0x500 
    cpu->mem[0x220] = 0xA1; cpu->mem[0x221] = 0x00; // N
    cpu->mem[0x222] = 0xD1; cpu->mem[0x223] = 0x05; 
    cpu->mem[0x224] = 0x2E; cpu->mem[0x225] = 0x00; //128 cycle pause

    cpu->mem[0x226] = 0xA1; cpu->mem[0x227] = 0x05; // O
    cpu->mem[0x228] = 0xD2; cpu->mem[0x229] = 0x05; 
    cpu->mem[0x22A] = 0x2E; cpu->mem[0x22B] = 0x00; //128 cycle pause

    cpu->mem[0x22C] = 0xA1; cpu->mem[0x22D] = 0x0A; // R
    cpu->mem[0x22E] = 0xD3; cpu->mem[0x22F] = 0x05;
    cpu->mem[0x230] = 0x2E; cpu->mem[0x231] = 0x00; //128 cycle pause

    cpu->mem[0x232] = 0xA1; cpu->mem[0x233] = 0x05; // O
    cpu->mem[0x234] = 0xD4; cpu->mem[0x235] = 0x05;
    cpu->mem[0x236] = 0x2E; cpu->mem[0x237] = 0x00; //128 cycle pause

    cpu->mem[0x238] = 0xA1; cpu->mem[0x239] = 0x0F; // M
    cpu->mem[0x23A] = 0xD5; cpu->mem[0x23B] = 0x05;
    cpu->mem[0x23C] = 0x2E; cpu->mem[0x23D] = 0x00; //128 cycle pause
    cpu->mem[0x23E] = 0x00; cpu->mem[0x23F] = 0xEE; //RET from SLOWDRW

// FLASH5
    cpu->mem[0x240] = 0x66; cpu->mem[0x241] = 0x00; // V[6] = 0
    cpu->mem[0x242] = 0x00; cpu->mem[0x243] = 0xE0; // CLS

    cpu->mem[0x244] = 0xA1; cpu->mem[0x245] = 0x00; // N
    cpu->mem[0x246] = 0xD1; cpu->mem[0x247] = 0x05; 

    cpu->mem[0x248] = 0xA1; cpu->mem[0x249] = 0x05; // O
    cpu->mem[0x24A] = 0xD2; cpu->mem[0x24B] = 0x05; 

    cpu->mem[0x24C] = 0xA1; cpu->mem[0x24D] = 0x0A; // R
    cpu->mem[0x24E] = 0xD3; cpu->mem[0x24F] = 0x05;

    cpu->mem[0x250] = 0xA1; cpu->mem[0x251] = 0x05; // O
    cpu->mem[0x252] = 0xD4; cpu->mem[0x253] = 0x05;

    cpu->mem[0x254] = 0xA1; cpu->mem[0x255] = 0x0F; // M
    cpu->mem[0x256] = 0xD5; cpu->mem[0x257] = 0x05;

    cpu->mem[0x258] = 0x2E; cpu->mem[0x259] = 0x00; //64 cycle pause
    cpu->mem[0x25A] = 0x76; cpu->mem[0x25B] = 0x01; //V[6]++
    cpu->mem[0x25C] = 0x00; cpu->mem[0x25D] = 0xE0; //CLS
    cpu->mem[0x25E] = 0x46; cpu->mem[0x25F] = 0x05; //Skip if V[6] != 5
    cpu->mem[0x260] = 0x00; cpu->mem[0x261] = 0xEE; //RET from FLASH5
    cpu->mem[0x262] = 0x2E; cpu->mem[0x263] = 0x00; //64 cycle pause
    cpu->mem[0x264] = 0x12; cpu->mem[0x265] = 0x44; //JP 0x244 (Draw N)
// END FLASH5 

*/

