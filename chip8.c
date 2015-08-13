#include <stdio.h>
#include <stdlib.h> /* for rand() */
#include <SDL.h>
#include <time.h>

#define W 64
#define H 32
#define max(A, B)  ((A) > (B) ? (A) : (B))

struct cpu {
  unsigned char mem[0x1000], V[16], DT, ST, gfx[W * H], keys[16], wait_key;
  unsigned short stack[16], I, PC, SP;
};

unsigned char fonts[80] =
{ 
  0xF0, 0x90, 0x90, 0x90, 0xF0, 0x20, 0x60, 0x20, 0x20, 0x70, 0xF0, 0x10, 0xF0, 0x80, 0xF0, 0xF0, 0x10, 0xF0, 0x10, 0xF0,
  0x90, 0x90, 0xF0, 0x10, 0x10, 0xF0, 0x80, 0xF0, 0x10, 0xF0, 0xF0, 0x80, 0xF0, 0x90, 0xF0, 0xF0, 0x10, 0x20, 0x40, 0x40,
  0xF0, 0x90, 0xF0, 0x90, 0xF0, 0xF0, 0x90, 0xF0, 0x10, 0xF0, 0xF0, 0x90, 0xF0, 0x90, 0x90, 0xE0, 0x90, 0xE0, 0x90, 0xE0,
  0xF0, 0x80, 0x80, 0x80, 0xF0, 0xE0, 0x90, 0x90, 0x90, 0xE0, 0xF0, 0x80, 0xF0, 0x80, 0xF0, 0xF0, 0x80, 0xF0, 0x80, 0x80
};
/* memcpy? *//*memset(cpu->V,0,16);*/
void load (const char* n, struct cpu* cpu){ FILE *f=fopen(n,"r"); fread(&(cpu->mem[0x200]),sizeof(char),0xe00,f); fclose(f); }
void font_init (struct cpu* cpu) { for (int i=0; i<80; i++) { cpu->mem[i] = fonts[i]; } }
void cpu_init  (struct cpu* cpu)
{ 
  cpu->PC = 0x200, cpu->I = 0, cpu->SP = 0, cpu->DT = 0, cpu->ST = 0, cpu->wait_key = 0;
  for(int i=0;i<0x1000;i++) { cpu->mem[i] = 0; } for(int i=0;i<H*W;i++) { cpu->gfx[i] = 0; }
  for(int i=0;i<16;i++) { cpu->V[i] = 0;    cpu->stack[i] = 0;    cpu->keys[i] = 0; } 
  font_init(cpu); 
}

#define params struct cpu* cpu, unsigned short op, \
unsigned short nnn, unsigned short n, unsigned short x, unsigned short y, unsigned short kk
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
void add_vx_kk (params) {    V[x] += kk;                                                              }
void or        (params) {    V[x] |= V[y];                                                            }
void and       (params) {    V[x] &= V[y];                                                            }
void xor       (params) {    V[x] ^= V[y];                                                            }
void rnd       (params) {    V[x]  = (rand() % 255) & kk;                                             }
void ld_vx_vy  (params) {    V[x]  = V[y];                                                            }
void ld_vx_kk  (params) {    V[x]  = kk;                                                              }
void ld_vx_dt  (params) {    V[x] = DT;                                                               }
void subb      (params) {    VF = ( V[x] > V[y] ) ? 1 : 0;    V[x] -= V[y];                           }
void shr       (params) {    VF = ( V[x] & 1 );    V[x] >>= 1;                                        }
void subnb     (params) {    VF = ( V[y] > V[x] ) ? 1 : 0;    V[x] = V[y] - V[x];                     }
void shl       (params) {    VF = V[x >> 3]; V[x] <<= 1;                                              }
void add_i_vx  (params) {    I += V[x];                                                               }
void ld_I_nnn  (params) {    I  = nnn;                                                                }
void ld_f_vx   (params) {    I  = V[x] * 5;  /* Fonts are 5 bytes, * 5 is the offset. */              }
void ret       (params) {    PC_ = stack[SP--];                                                       }
void jp_nnn    (params) {    PC_ = nnn;                                                               }
void jp_v0_nnn (params) {    PC_ = nnn + V[0];                                                        }
void ld_dt_vx  (params) {    DT = V[x];                                                               }
void ld_st_vx  (params) {    ST = V[x];                                                               }
void bcd       (params) {    mem[I] = V[x] / 100; mem[I+1] = (V[x] % 100) / 10; mem[I+2] = V[x] % 10; }
void addc      (params) {    n = V[x] + V[y];    VF = (n>0xFF) ? 1 : 0; V[x] = n & 0xFF;              }
void call_nnn  (params) {    stack[++SP] = PC_;    PC_ = nnn;                                         }
void ld_vx_k   (params) {    cpu->wait_key = x | 0x80;                                                }
void se_vx_kk  (params) {    if (V[x] == kk) PC_ += 2;                                                }
void sne_vx_kk (params) {    if (V[x] != kk) PC_ += 2;                                                }
void se_vx_vy  (params) {    if (V[x] == V[y]) PC_ += 2;                                              }
void sne_vx_vy (params) {    if (V[x] != V[y]) PC_ += 2;                                              }
void skp_vx    (params) {    if (   cpu->keys[ V[x] ] )  PC_ += 2;                                    }
void sknp_vx   (params) {    if (! (cpu->keys[ V[x] ]) ) PC_ += 2;                                    }
void cls       (params) {    for (int i=0; i< W*H; i++) { cpu->gfx[i] = 0; }                          }
void store_reg (params) {    for (int i=0; i<=x; i++) { mem[I + i] = V[i]; }                          }
void read_reg  (params) {    for (int i=0; i<=x; i++) { V[i] = mem[I + i]; }                          }
/* use entire byte, handle wrapping wrapping */
void drw       (params)
{
  unsigned short xcoord = V[x], ycoord = V[y];
  unsigned short height = n, pixel; int xline, yline;
  VF = 0;
  for (yline = 0; yline < height; yline++) {
      pixel = mem[I + yline];
      for (xline = 0; xline < 8; xline++) {
	  if ((pixel & (0x80 >> xline)) != 0) {
	      if (cpu->gfx[(xcoord + xline + ((ycoord + yline) * 64))] == 1)
		VF = 1;                                 
	      cpu->gfx[xcoord + xline + ((ycoord + yline) * 64)] ^= 1;
	    }
	}
    }
}

void (*zero_table[2]) (params) = { cls, ret };
void (*alu_table[9])  (params) = { ld_vx_vy, or, and, xor, addc, subb, shr, subnb, shl };
void (*e_table[2])    (params) = { skp_vx, sknp_vx };
void (*f_table[9])    (params) = { add_i_vx, ld_vx_k, read_reg, ld_dt_vx, store_reg, ld_f_vx, ld_st_vx, ld_vx_dt, bcd };
void zero (params) { zero_table[(n) ? 1 : 0] (cpu, op, nnn, n, x, y, kk); }
void alu  (params) { alu_table[ ((n)==0xe) ? 8 : (n) ] (cpu, op, nnn, n, x, y, kk); }
void e    (params) { e_table[ ((n)==0xe) ? 0 : (n) ] (cpu, op, nnn, n, x, y, kk); }
void f    (params) { unsigned u=kk%9; f_table[((u==3)||(u==6))?((u==3)?((n==5)?3:0):((n==8)?6:8)):u](cpu,op,nnn,n,x,y,kk); }
void (*chip8_table[16]) (params) =
{ zero, jp_nnn, call_nnn, se_vx_kk, sne_vx_kk, se_vx_vy, ld_vx_kk, 
  add_vx_kk, alu, sne_vx_vy, ld_I_nnn, jp_v0_nnn, rnd, drw, e, f };
void exec (struct cpu* cpu)
{ 
  unsigned short ins = (mem[PC_] << 8) | (mem[PC_+1]);    PC_ += 2; 
  unsigned short op  = (ins >>12) & 0xF;    unsigned short nnn = (ins >> 0) & 0xFFF; unsigned short n   = (ins >> 0) & 0xF;
  unsigned short x   = (ins >> 8) & 0xF;    unsigned short y   = (ins >> 4) & 0xF;   unsigned short kk  = (ins >> 0) & 0xFF;
  chip8_table[op] (cpu, op, nnn, n, x, y, kk); 
}

#undef params
#undef PC_ cpu->PC
#undef SP cpu->SP
#undef I cpu->I
#undef DT cpu->DT
#undef ST cpu->ST
#undef mem cpu->mem
#undef V cpu->V
#undef stack cpu->stack

int main (int argc, char** argv)
{
  struct cpu* cpu = (struct cpu*)malloc(sizeof(struct cpu));    cpu_init(cpu);
  if (argc>1) load(argv[1], cpu); else { printf("No rom, exiting\n"); exit(1); }
 
  /* Create a screen. */
  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window *window = SDL_CreateWindow(argv[1], SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 512, 256, NULL);
  SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
  
  unsigned insns_per_frame = 50, max_consecutive_insns = 0;    int frames_done = 0, running = 1;
  struct timespec start_t; clock_gettime(CLOCK_MONOTONIC_RAW, &start_t);
  
  while(running)
    { 
      for(unsigned a=0; a<max_consecutive_insns && !(cpu->wait_key & 0x80); ++a)
	exec(cpu);

      for(SDL_Event ev; SDL_PollEvent(&ev); )
	switch(ev.type)
	  {
	    unsigned k;
	  case SDL_QUIT: running = 0; break;
	  case SDL_KEYDOWN:
	  case SDL_KEYUP:
	    switch (ev.key.keysym.sym)
	      {
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
	      }
	    if(ev.type==SDL_KEYDOWN && (cpu->wait_key & 0x80))
	      {
		cpu->wait_key        &= 0x7F;
		cpu->V[cpu->wait_key] = k;
	      }
	  }
    
      // Check how many frames we are _supposed_ to have rendered so far
      struct timespec cur_t;    clock_gettime(CLOCK_MONOTONIC_RAW, &cur_t);
      double elapsed = (( 1000.0*cur_t.tv_sec + 1e-6*cur_t.tv_nsec) - (1000.0*start_t.tv_sec + 1e-6*start_t.tv_nsec))/1000;
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
	  destRect->x = 0;    destRect->y = 0;    destRect->w = 8;    destRect->h = 8;
	  for (int y = 0; y < H; y++)
	    for (int x = 0; x < W; x++)
	      if (cpu->gfx[(y * 64) + x] == 1) {
		destRect->x = x * 8;
		destRect->y = y * 8;
		SDL_RenderFillRect(renderer, destRect);
	      }   	    
	  free(destRect);
	  SDL_RenderPresent(renderer);
        }
      // Adjust the instruction count to compensate for our rendering speed
      max_consecutive_insns = max(frames, 1) * insns_per_frame;
      // If the CPU is still waiting for a key, or if we didn't have a frame yet, consume a bit of time
      if((cpu->wait_key & 0x80) || !frames) SDL_Delay(1000/60);
    }
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  free(cpu);
  
  return 0;
}

/*
void cls       (params) {    for (int i=0; i< W*H; i++) { cpu->gfx[i] = 0; }                          }
void ret       (params) {    PC_ = stack[SP--];                                                       }
void jp_nnn    (params) {    PC_ = nnn;                                                               }
void call_nnn  (params) {    stack[++SP] = PC_;    PC_ = nnn;                                         }
void se_vx_kk  (params) {    if (V[x] == kk) PC_ += 2;                                                }
void sne_vx_kk (params) {    if (V[x] != kk) PC_ += 2;                                                }
void se_vx_vy  (params) {    if (V[x] == V[y]) PC_ += 2;                                              }
void ld_vx_kk  (params) {    V[x]  = kk;                                                              }
void add_vx_kk (params) {    V[x] += kk;                                                              }
void sne_vx_vy (params) {    if (V[x] != V[y]) PC_ += 2;                                              }
void ld_I_nnn  (params) {    I  = nnn;                                                                }
void jp_v0_nnn (params) {    PC_ = nnn + V[0];                                                        }
void rnd       (params) {    V[x]  = (rand() % 255) & kk;                                             }
void ld_vx_vy  (params) {    V[x]  = V[y];                                                            }
void or        (params) {    V[x] |= V[y];                                                            }
void and       (params) {    V[x] &= V[y];                                                            }
void xor       (params) {    V[x] ^= V[y];                                                            }
void addc      (params) {    n = V[x] + V[y];    VF = (n>0xFF) ? 1 : 0; V[x] = n & 0xFF;              }
void subb      (params) {    VF = ( V[x] > V[y] ) ? 1 : 0;    V[x] -= V[y];                           }
void shr       (params) {    VF = ( V[x] & 1 );    V[x] >>= 1;                                        }
void subnb     (params) {    VF = ( V[y] > V[x] ) ? 1 : 0;    V[x] = V[y] - V[x];                     }
void shl       (params) {    VF = V[x >> 3]; V[x] <<= 1;                                              }
void skp_vx    (params) {    if (   cpu->keys[ V[x] ] )  PC_ += 2;                                    }
void sknp_vx   (params) {    if (! (cpu->keys[ V[x] ]) ) PC_ += 2;                                    }
void ld_vx_dt  (params) {    V[x] = DT;                                                               }
void ld_vx_k   (params) {    cpu->wait_key = x | 0x80;                                                }
void ld_dt_vx  (params) {    DT = V[x];                                                               }
void ld_st_vx  (params) {    ST = V[x];                                                               }
void add_i_vx  (params) {    I += V[x];                                                               }
void ld_f_vx   (params) {    I  = V[x] * 5;                                                           }
void bcd       (params) {    mem[I] = V[x] / 100; mem[I+1] = (V[x] % 100) / 10; mem[I+2] = V[x] % 10; }
void store_reg (params) {    for (int i=0; i<=x; i++) { mem[I + i] = V[i]; }                          }
void read_reg  (params) {    for (int i=0; i<=x; i++) { V[i] = mem[I + i]; }                          }
*/

/*void (*f_table[9])    (params) = { ld_vx_dt, ld_vx_k, ld_dt_vx, ld_st_vx, add_i_vx, ld_f_vx, bcd, store_reg, read_reg };*/
/*
void f    (params)
{
  int i;
  switch (kk)
    { case 0x07: i=0; break; case 0x0a: i=1; break; case 0x15: i=2; break;
      case 0x18: i=3; break; case 0x1e: i=4; break; case 0x29: i=5; break; 
      case 0x33: i=6; break; case 0x55: i=7; break; case 0x65: i=8; break; }
  f_table[i] (cpu, op, nnn, n, x, y, kk);
}
*/
