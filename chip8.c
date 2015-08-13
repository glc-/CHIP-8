#include <stdio.h>
#include <stdlib.h> /* for rand() */
#include <SDL.h>
#include <time.h>

#define W 64
#define H 32
#define max(a,b)				\
  ({ __typeof__ (a) _a = (a);			\
    __typeof__ (b) _b = (b);			\
    _a > _b ? _a : _b; })

struct cpu {
  unsigned char mem[0x1000], V[16];
  unsigned short stack[16];
  unsigned short I, PC, SP;
  unsigned char DT, ST; /* Delay Timer, Sound Timer */
  unsigned char gfx[W * H], keys[16], wait_key;
};

unsigned char fonts[80] =
  { 
    0xF0, 0x90, 0x90, 0x90, 0xF0, // 0
    0x20, 0x60, 0x20, 0x20, 0x70, // 1
    0xF0, 0x10, 0xF0, 0x80, 0xF0, // 2
    0xF0, 0x10, 0xF0, 0x10, 0xF0, // 3
    0x90, 0x90, 0xF0, 0x10, 0x10, // 4
    0xF0, 0x80, 0xF0, 0x10, 0xF0, // 5
    0xF0, 0x80, 0xF0, 0x90, 0xF0, // 6
    0xF0, 0x10, 0x20, 0x40, 0x40, // 7
    0xF0, 0x90, 0xF0, 0x90, 0xF0, // 8
    0xF0, 0x90, 0xF0, 0x10, 0xF0, // 9
    0xF0, 0x90, 0xF0, 0x90, 0x90, // A
    0xE0, 0x90, 0xE0, 0x90, 0xE0, // B
    0xF0, 0x80, 0x80, 0x80, 0xF0, // C
    0xE0, 0x90, 0x90, 0x90, 0xE0, // D
    0xF0, 0x80, 0xF0, 0x80, 0xF0, // E
    0xF0, 0x80, 0xF0, 0x80, 0x80  // F
  };

void font_init (struct cpu* cpu)
{
  for (int i=0; i<80; i++)
    cpu->mem[i] = fonts[i];
}

void cpu_init (struct cpu* cpu)
{
  cpu->PC = 0x200;
  cpu->I = 0;
  cpu->SP = 0;
  cpu->DT = 0;
  cpu->ST = 0;
  cpu->wait_key = 0;

  for(int i=0;i<0x1000;i++)
    cpu->mem[i] = 0;

  for(int i=0;i<H*W;i++)
    cpu->gfx[i] = 0;

  for(int i=0;i<16;i++) {
    cpu->V[i] = 0;
    cpu->stack[i] = 0;
    cpu->keys[i] = 0;
  }
  font_init(cpu);
}

void load (const char* filename, struct cpu* cpu)
{
  unsigned short pos = 0x200;
  FILE *fp = fopen(filename, "r");
  fread( &(cpu->mem[pos++ & 0xFFF]), sizeof(unsigned char), 0x1000-0x200, fp);
  fclose(fp);
}

#define params struct cpu* cpu, unsigned short op, unsigned short nnn, unsigned short n, unsigned short x, unsigned short y, unsigned short kk
/* Instructions */
void cls       (params) { for (int i=0; i< W*H; i++) { cpu->gfx[i] = 0; } }
void ret       (params) { cpu->PC = cpu->stack[cpu->SP--]; }
void jp_nnn    (params) { cpu->PC = nnn; }
void call_nnn  (params) { cpu->stack[++cpu->SP] = cpu->PC; cpu->PC = nnn; }
void se_vx_kk  (params) { if (cpu->V[x] == kk) cpu->PC += 2; }
void sne_vx_kk (params) { if (cpu->V[x] != kk) cpu->PC += 2; }
void se_vx_vy  (params) { if (cpu->V[x] == cpu->V[y]) cpu->PC += 2; }
void ld_vx_kk  (params) { cpu->V[x]  = kk; }
void add_vx_kk (params) { cpu->V[x] += kk; }
void sne_vx_vy (params) { if (cpu->V[x] != cpu->V[y]) cpu->PC += 2; }
void ld_I_nnn  (params) { cpu->I  = nnn; }
void jp_v0_nnn (params) { cpu->PC = nnn + cpu->V[0]; }
void rnd       (params) { cpu->V[x]  = (rand() % 255) & kk; }
void ld_vx_vy  (params) { cpu->V[x]  = cpu->V[y]; }
void or        (params) { cpu->V[x] |= cpu->V[y]; }
void and       (params) { cpu->V[x] &= cpu->V[y]; }
void xor       (params) { cpu->V[x] ^= cpu->V[y]; }
void addc      (params) { unsigned short tmp = cpu->V[x] + cpu->V[y]; cpu->V[0xF] = (tmp>0xFF) ? 1 : 0;cpu->V[x] = tmp & 0xFF; }
void subb      (params) { cpu->V[0xF] = ( cpu->V[x] > cpu->V[y] ) ? 1 : 0; cpu->V[x] -= cpu->V[y]; }
void shr       (params) { cpu->V[0xF] = ( cpu->V[x] & 1 ); cpu->V[x] >>= 1; }
void subnb     (params) { cpu->V[0xF] = ( cpu->V[y] > cpu->V[x] ) ? 1 : 0; cpu->V[x] = cpu->V[y] - cpu->V[x]; }
void shl       (params) { cpu-> V[0xF] = cpu->V[x >> 3]; cpu->V[x] <<= 1; }
void skp_vx    (params) { if (   cpu->keys[ cpu->V[x] ] )  cpu->PC+=2; }
void sknp_vx   (params) { if (! (cpu->keys[ cpu->V[x] ]) ) cpu->PC+=2; }
void ld_vx_dt  (params) { cpu->V[x] = cpu->DT; }
void ld_vx_k   (params) { cpu->wait_key = x | 0x80; }
void ld_dt_vx  (params) { cpu->DT = cpu->V[x]; }
void ld_st_vx  (params) { cpu->ST = cpu->V[x]; }
void add_i_vx  (params) { cpu->I += cpu->V[x]; }
void ld_f_vx   (params) { cpu->I  = cpu->V[x] * 5; } /* Fonts are 5 bytes, *5 is the offset. */
void bcd       (params) { cpu->mem[cpu->I] = cpu->V[x] / 100; cpu->mem[cpu->I+1] = (cpu->V[x] % 100) / 10; cpu->mem[cpu->I+2] = cpu->V[x] % 10; }
void store_reg (params) { for (int i=0; i<=x; i++) { cpu->mem[cpu->I + i] = cpu->V[i]; } }
void read_reg  (params) { for (int i=0; i<=x; i++) { cpu->V[i] = cpu->mem[cpu->I + i]; } }
void drw       (params)
{
  unsigned short xcoord = cpu->V[x], ycoord = cpu->V[y];
  unsigned short height = n, pixel; int xline, yline;
  cpu->V[0xF] = 0;
  for (yline = 0; yline < height; yline++) {
      pixel = cpu->mem[cpu->I + yline];
      for (xline = 0; xline < 8; xline++) {
	  if ((pixel & (0x80 >> xline)) != 0) {
	      if (cpu->gfx[(xcoord + xline + ((ycoord + yline) * 64))] == 1)
		cpu->V[0xF] = 1;                                 
	      cpu->gfx[xcoord + xline + ((ycoord + yline) * 64)] ^= 1;
	    }
	}
    }
}

void (*zero_table[2]) (params) = { cls, ret };
void (*alu_table[9])  (params) = { ld_vx_vy, or, and, xor, addc, subb, shr, subnb, shl };
void (*e_table[2])    (params) = { skp_vx, sknp_vx };
void (*f_table[9])    (params) = { ld_vx_dt, ld_vx_k, ld_dt_vx, ld_st_vx, add_i_vx, ld_f_vx, bcd, store_reg, read_reg };

void zero (params) { zero_table[(n) ? 1 : 0] (cpu, op, nnn, n, x, y, kk); }
void alu  (params) { alu_table[ ((n)==0xe) ? 8 : (n) ] (cpu, op, nnn, n, x, y, kk); }
void e    (params) { e_table[ ((n)==0xe) ? 0 : (n) ] (cpu, op, nnn, n, x, y, kk); }
void f    (params)
{
  int foo;
  switch (kk)
    {
    case 0x0007: foo=0; break;
    case 0x000a: foo=1; break;
    case 0x0015: foo=2; break;
    case 0x0018: foo=3; break;
    case 0x001e: foo=4; break;
    case 0x0029: foo=5; break;
    case 0x0033: foo=6; break;
    case 0x0055: foo=7; break;
    case 0x0065: foo=8; break;
    }
  f_table[foo] (cpu, op, nnn, n, x, y, kk);
}

void (*chip8_table[16]) (params) =
{
  zero, jp_nnn, call_nnn, se_vx_kk, sne_vx_kk, se_vx_vy, ld_vx_kk, add_vx_kk, alu, sne_vx_vy, ld_I_nnn, jp_v0_nnn, rnd, drw, e, f
};


void exec (struct cpu* cpu)
{
  unsigned short ins = (cpu->mem[cpu->PC] << 8) | (cpu->mem[cpu->PC+1]);
  cpu->PC += 2; 

  unsigned short op  = (ins >>12) & 0xF;
  unsigned short nnn = (ins >> 0) & 0xFFF;
  unsigned short n   = (ins >> 0) & 0xF;
  unsigned short x   = (ins >> 8) & 0xF;  
  unsigned short y   = (ins >> 4) & 0xF;
  unsigned short kk  = (ins >> 0) & 0xFF;

  chip8_table[op] (cpu, op, nnn, n, x, y, kk);
}

#undef params

int main (int argc, char** argv)
{
  struct cpu* cpu = (struct cpu*)malloc(sizeof(struct cpu));
  cpu_init(cpu);
  load(argv[1], cpu);
 
  /* Create a screen. */
  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window *window = SDL_CreateWindow(argv[1], SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 512, 256, NULL);
  SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
  
  unsigned insns_per_frame = 50;
  unsigned max_consecutive_insns = 0;
  int frames_done = 0;
  struct timespec start_t;
  clock_gettime(CLOCK_MONOTONIC_RAW, &start_t);
  int running=1;
  while(running)
    { 
      for(unsigned a=0; a<max_consecutive_insns && !(cpu->wait_key & 0x80); ++a)
	exec(cpu);

      for(SDL_Event ev; SDL_PollEvent(&ev); )
	switch(ev.type)
	  {
	    unsigned short tmp;
	  case SDL_QUIT: running = 0; break;
	  case SDL_KEYDOWN:
	  case SDL_KEYUP:
	    switch (ev.key.keysym.sym)
	      {
	      case SDLK_1: cpu->keys[0x1] = ev.type==SDL_KEYDOWN; if (cpu->keys[0x1]){tmp=1;} break;
	      case SDLK_2: cpu->keys[0x2] = ev.type==SDL_KEYDOWN; if (cpu->keys[0x2]){tmp=1;} break;
	      case SDLK_3: cpu->keys[0x3] = ev.type==SDL_KEYDOWN; if (cpu->keys[0x3]){tmp=1;} break;
	      case SDLK_4: cpu->keys[0xC] = ev.type==SDL_KEYDOWN; if (cpu->keys[0xC]){tmp=1;} break;
	      case SDLK_q: cpu->keys[0x4] = ev.type==SDL_KEYDOWN; if (cpu->keys[0x4]){tmp=1;} break;
	      case SDLK_w: cpu->keys[0x5] = ev.type==SDL_KEYDOWN; if (cpu->keys[0x5]){tmp=1;} break;
	      case SDLK_e: cpu->keys[0x6] = ev.type==SDL_KEYDOWN; if (cpu->keys[0x6]){tmp=1;} break;
	      case SDLK_r: cpu->keys[0xD] = ev.type==SDL_KEYDOWN; if (cpu->keys[0xD]){tmp=1;} break;
	      case SDLK_a: cpu->keys[0x7] = ev.type==SDL_KEYDOWN; if (cpu->keys[0x7]){tmp=1;} break;
	      case SDLK_s: cpu->keys[0x8] = ev.type==SDL_KEYDOWN; if (cpu->keys[0x8]){tmp=1;} break;
	      case SDLK_d: cpu->keys[0x9] = ev.type==SDL_KEYDOWN; if (cpu->keys[0x9]){tmp=1;} break;
	      case SDLK_f: cpu->keys[0xE] = ev.type==SDL_KEYDOWN; if (cpu->keys[0xE]){tmp=1;} break;
	      case SDLK_z: cpu->keys[0xA] = ev.type==SDL_KEYDOWN; if (cpu->keys[0xA]){tmp=1;} break;
	      case SDLK_x: cpu->keys[0x0] = ev.type==SDL_KEYDOWN; if (cpu->keys[0x0]){tmp=1;} break;
	      case SDLK_c: cpu->keys[0xB] = ev.type==SDL_KEYDOWN; if (cpu->keys[0xB]){tmp=1;} break;
	      case SDLK_v: cpu->keys[0xF] = ev.type==SDL_KEYDOWN; if (cpu->keys[0xF]){tmp=1;} break;
	      case SDLK_5: cpu->keys[0x5] = ev.type==SDL_KEYDOWN; if (cpu->keys[0x5]){tmp=1;} break;
	      case SDLK_6: cpu->keys[0x6] = ev.type==SDL_KEYDOWN; if (cpu->keys[0x6]){tmp=1;} break;
	      case SDLK_7: cpu->keys[0x7] = ev.type==SDL_KEYDOWN; if (cpu->keys[0x7]){tmp=1;} break;
	      case SDLK_8: cpu->keys[0x8] = ev.type==SDL_KEYDOWN; if (cpu->keys[0x8]){tmp=1;} break;
	      case SDLK_9: cpu->keys[0x9] = ev.type==SDL_KEYDOWN; if (cpu->keys[0x9]){tmp=1;} break;
	      case SDLK_0: cpu->keys[0x0] = ev.type==SDL_KEYDOWN; if (cpu->keys[0x0]){tmp=1;} break;
	      case SDLK_ESCAPE: running=0; break;
	      }
	    if(ev.type==SDL_KEYDOWN && (cpu->wait_key & 0x80))
	      {
		cpu->wait_key        &= 0x7F;
		cpu->V[cpu->wait_key] = tmp;
	      }
	  }
    
      // Check how many frames we are _supposed_ to have rendered so far
      struct timespec cur_t;
      clock_gettime(CLOCK_MONOTONIC_RAW, &cur_t);
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
	    
	  //SDL_Rect *destRect = new SDL_Rect;
	  SDL_Rect* destRect = (SDL_Rect *)malloc(sizeof(SDL_Rect));
	  destRect->x = 0;
	  destRect->y = 0;
	  destRect->w = 8;
	  destRect->h = 8;
	  int x,y;  
	  for (y = 0; y < H; y++)
	    for (x = 0; x < W; x++)
	      if (cpu->gfx[(y * 64) + x] == 1) {
		destRect->x = x * 8;
		destRect->y = y * 8;

		SDL_RenderFillRect(renderer, destRect);
	      }
	    	    
	  free (destRect);
	  SDL_RenderPresent(renderer);
        }
      // Adjust the instruction count to compensate for our rendering speed
      max_consecutive_insns = max(frames, 1) * insns_per_frame;
      // If the CPU is still waiting for a key, or if we didn't
      // have a frame yet, consume a bit of time
      if((cpu->wait_key & 0x80) || !frames) SDL_Delay(1000/60);
    }
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  
  return 0;
}
