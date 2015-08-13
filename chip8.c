#include <stdlib.h> /* for rand() */
#include <SDL.h>
#include <time.h>
#include <math.h>
#include <sys/sysinfo.h>
#include <stddef.h>

#define W 64
#define H 32

struct cpu {
  unsigned char mem[0x1000], V[16];
  unsigned short stack[16];
  unsigned short I, PC, SP;
  unsigned char DT, ST; /* Delay Timer, Sound Timer */
  unsigned char gfx[W * H], keys[16], wait_key;
};

unsigned char chip8_fontset[80] =
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


void font (struct cpu* cpu)
{
  int i;
  for (i=0; i<80; i++)
    cpu->mem[i] = chip8_fontset[i];
}

void cpu_init(struct cpu* cpu)
{
  cpu->PC = 0x200;
  cpu->I = 0;
  cpu->SP = 0;
  cpu->DT = 0;
  cpu->ST = 0;
  cpu->wait_key = 0;

  int i;
  for(i=0;i<0x1000;i++)
    cpu->mem[i] = 0;

  for(i=0;i<H*W;i++)
    cpu->gfx[i] = 0;

  for(i=0;i<16;i++) {
    cpu->V[i] = 0;
    cpu->stack[i] = 0;
    cpu->keys[i] = 0;
  }
  font(cpu);
}

void load(const char* filename, struct cpu* cpu)
{
  unsigned short pos = 0x200;
  FILE *fp = fopen(filename, "r");
  fread(&(cpu->mem[pos++ & 0xFFF]), sizeof(unsigned char), 0x1000-0x200, fp);
  fclose(fp);
}


void exec (struct cpu* cpu)
{
  unsigned short opcode = (cpu->mem[cpu->PC] << 8) | (cpu->mem[cpu->PC+1]);
  cpu->PC+=2;
  
  switch (opcode & 0xf000) 
    {
    case 0x0000: 
      { 
	switch (opcode & 0x00ff)
	  {
	  case 0x00e0:
	    { 
	      int i;
	      for (i=0; i< W*H; i++) 
		{ 
		  cpu->gfx[i] = 0; 
		}
	    }
	    break;
 
	  case 0x00ee:
	    {
	      cpu->PC = cpu->stack[cpu->SP--];
	    }
	    break;
	  default:
	    {
	      printf("Invalid opcode\n");
	    }
	    break;
	  } /*end 0x00ff switch */
      } /* end case 0x0000 */
      break;

    case 0x1000:  
      {
	cpu->PC = (opcode & 0x0fff); 
	
      }
      break;

    case 0x2000: 
      { 
	cpu->stack[++cpu->SP] = cpu->PC;
	cpu->PC = (opcode & 0x0fff);
      }
      break;

    case 0x3000:
      {
	if (cpu->V[(opcode & 0x0f00) >> 8] == (opcode & 0x00ff))
	  cpu->PC+=2;
      }
      break;

    case 0x4000:
      {
	if (cpu->V[(opcode & 0x0f00) >> 8] != (opcode & 0x00ff))
	  cpu->PC+=2;
      }
      break;

    case 0x5000: 
      {
	if (cpu->V[(opcode & 0x0f00) >> 8] == cpu->V[(opcode & 0x00f0) >> 4])
	  {
	    cpu->PC+=2;
	  }
      }
      break;

    case 0x6000:
      {
	cpu->V[(opcode & 0x0f00) >> 8] = (opcode & 0x00ff); 
      }
      break;

    case 0x7000:
      {
	cpu->V[(opcode & 0x0f00) >> 8] = cpu->V[(opcode & 0x0f00) >> 8] + (opcode & 0x00ff); 
      }
      break;

    case 0x8000: 
      {
	switch (opcode & 0x000f) 
	  {
	  case 0x0000:
	    {
	      cpu->V[(opcode & 0x0f00) >> 8] = cpu->V[(opcode & 0x00f0) >> 4]; 
	    } 
	    break;

	  case 0x0001:
	    { 
	      cpu->V[(opcode & 0x0f00) >> 8] = cpu->V[(opcode & 0x0f00) >> 8] | cpu->V[(opcode & 0x00f0) >> 4];
	    }
	    break;

	  case 0x0002:
	    {
	      cpu->V[(opcode & 0x0f00) >> 8] = cpu->V[(opcode & 0x0f00) >> 8] & cpu->V[(opcode & 0x00f0) >> 4]; 
	    }
	    break;

	  case 0x0003:
	    {
	      cpu->V[(opcode & 0x0f00) >> 8] = cpu->V[(opcode & 0x0f00) >> 8] ^ cpu->V[(opcode & 0x00f0) >> 4];
	    }
	    break;

	  case 0x0004:
	    {
	      unsigned short tmp = cpu->V[(opcode & 0x0f00) >> 8] + cpu->V[(opcode & 0x00f0) >> 4];
	      if ( tmp > 255 ) {
		cpu->V[0xf] = 1;
	      } else {
		cpu->V[0xf] = 0;
	      }
	      cpu->V[(opcode & 0x0f00) >> 8] = (tmp & 0x00ff);
	    }
	    break;   
  
	  case 0x0005:
	    {
	      if ( cpu->V[(opcode & 0x0f00) >> 8] > cpu->V[(opcode & 0x00f0) >> 4] ) {
		cpu->V[0xf] = 1;
	      } else {
		cpu->V[0xf] = 0;
	      }
	      cpu->V[(opcode & 0x0f00) >> 8] -= cpu->V[(opcode & 0x00f0) >> 4];
	    }
	    break;

	  case 0x0006:
	    {
	      cpu->V[0xf] = ( cpu->V[(opcode & 0x0f00) >> 8] & 1 );
	      cpu->V[(opcode & 0x0f00) >> 8] >>= 1;
	    }
	    break;

	  case 0x0007:
	    {
	      if ( cpu->V[(opcode & 0x00f0) >> 4] > cpu->V[(opcode & 0x0f00) >> 8] ) {
		cpu->V[0xf] = 1;
	      } else {
		cpu->V[0xf] = 0;
	      }
	      cpu->V[(opcode & 0x0f00) >> 8] = cpu->V[(opcode & 0x00f0) >> 4] - cpu->V[(opcode & 0x0f00) >> 8];
	    }
	    break;

	  case 0x000e:
	    {
	      cpu-> V[0xf] = cpu->V[(opcode & 0x0f00) >> 11];
	      cpu->V[(opcode & 0x0f00) >> 8] <<= 1;
	    }
	    break;
	  
	  default:
	    {
	      printf("Invalid opcode\n");
	    }
	    break;
	  } /* end switch (opcode & 0x000f) */
      } /* end case 0x8000 */
      break;

    case 0x9000:
      {
	if (cpu->V[(opcode & 0x0f00) >> 8] != cpu->V[(opcode & 0x00f0) >> 4])
	  {
	    cpu->PC+=2;
	  } 
      }

      break;
    case 0xa000:
      {
	cpu->I = (opcode & 0x0fff);
      }
      break;

    case 0xb000:
      {
	cpu->PC = (opcode & 0x0fff) + cpu->V[0]; 
      }
      break;

    case 0xc000:
      {
	cpu->V[(opcode & 0x0f00) >> 8] = (rand() % 255) & (opcode & 0x00ff);
      }
      break;

    case 0xd000:
      {
	unsigned short x = cpu->V[(opcode & 0x0F00) >> 8];
	unsigned short y = cpu->V[(opcode & 0x00F0) >> 4];
	unsigned short height = opcode & 0x000F;
	unsigned short pixel;
     
	int xline, yline;
	  
	cpu->V[0xF] = 0;
	for (yline = 0; yline < height; yline++)
	  {
	    pixel = cpu->mem[cpu->I + yline];
	    for(xline = 0; xline < 8; xline++)
	      {
		if((pixel & (0x80 >> xline)) != 0)
		  {
		    if(cpu->gfx[(x + xline + ((y + yline) * 64))] == 1)
		      cpu->V[0xF] = 1;                                 
		    cpu->gfx[x + xline + ((y + yline) * 64)] ^= 1;
		  }
	      }
	  }
      }
      break;

    case 0xe000:
      {
	switch (opcode & 0x000f)
	  {
	  case 0x000e:
	    {
	      if ( cpu->keys[ cpu->V[(opcode & 0x0F00) >> 8]&15] )
		cpu->PC+=2;
	    }
	    break;

	  case 0x0001: 
	    {
	      if (! (cpu->keys[ cpu->V[(opcode & 0x0F00) >> 8]&15] ) )
		cpu->PC+=2;
	    }
	    break;
	  }
      }
      break;

    case 0xf000:
      {
	switch (opcode & 0x00ff)
	  {
	  case 0x0007:
	    {
	      cpu->V[(opcode & 0x0f00) >> 8] = cpu->DT;
	    }
	    break;

	  case 0x000a:
	    {
	      cpu->wait_key = ((opcode & 0x0f00) >> 8 ) | 0x80;
	    }
	    break;
 
	  case 0x0015:
	    {
	      cpu->DT = cpu->V[(opcode & 0x0f00) >> 8];
	    }
	    break;

	  case 0x0018:
	    {
	      cpu->ST = cpu->V[(opcode & 0x0f00) >> 8];
	    }
	    break;

	  case 0x001e:
	    {
	      cpu->I += cpu->V[(opcode & 0x0f00) >> 8];
	    }
	    break;

	  case 0x0029:
	    {
	      cpu->I = cpu->V[(opcode & 0x0f00) >> 8] * 5;
	    }
	    break; /* Fonts start at address 0x0000 and occupy 5 bytes, so *5 is for the offset. */

	  case 0x0033:
	    { 
	      cpu->mem[cpu->I] = cpu->V[(opcode & 0x0f00) >> 8] / 100;
	      cpu->mem[cpu->I+1] = ( cpu->V[(opcode & 0x0f00) >> 8] % 100 ) / 10;
	      cpu->mem[cpu->I+2] = cpu->V[(opcode & 0x0f00) >> 8] % 10;
	    }
	    break;

	  case 0x0055:
	    {
	      int index;
	      for (index=0; index <= ( (opcode & 0x0f00) >> 8 ); index++) {
		cpu->mem[cpu->I + index] = cpu->V[index];
	      }
	    }
	    break;

	  case 0x0065:
	    {
	      int index;
	      for (index=0; index <= ( (opcode & 0x0f00) >> 8 ); index++) {
		cpu->V[index] =  cpu->mem[cpu->I + index];
	      }
      	    }
	    break;
	  }
      }
      break;
    default:
      {
	printf("Invalid opcode\n");
      }
      break;
    }
} 



int main (int argc, char** argv)
{
  struct cpu cpu;
  cpu_init(&cpu);
  load(argv[1], &cpu);
 
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
      for(unsigned a=0; a<max_consecutive_insns && !(cpu.wait_key & 0x80); ++a)
     	  exec(&cpu);

      for(SDL_Event ev; SDL_PollEvent(&ev); )
            switch(ev.type)
            {
	      unsigned short tmp;
                case SDL_QUIT: running = 0; break;
                case SDL_KEYDOWN:
                case SDL_KEYUP:
		  switch (ev.key.keysym.sym)
		    {
		    case SDLK_1: cpu.keys[0x1] = ev.type==SDL_KEYDOWN; if (cpu.keys[0x1]){tmp=1;} break;
		    case SDLK_2: cpu.keys[0x2] = ev.type==SDL_KEYDOWN; if (cpu.keys[0x2]){tmp=1;} break;
		    case SDLK_3: cpu.keys[0x3] = ev.type==SDL_KEYDOWN; if (cpu.keys[0x3]){tmp=1;} break;
		    case SDLK_4: cpu.keys[0xC] = ev.type==SDL_KEYDOWN; if (cpu.keys[0xC]){tmp=1;} break;
		    case SDLK_q: cpu.keys[0x4] = ev.type==SDL_KEYDOWN; if (cpu.keys[0x4]){tmp=1;} break;
		    case SDLK_w: cpu.keys[0x5] = ev.type==SDL_KEYDOWN; if (cpu.keys[0x5]){tmp=1;} break;
		    case SDLK_e: cpu.keys[0x6] = ev.type==SDL_KEYDOWN; if (cpu.keys[0x6]){tmp=1;} break;
		    case SDLK_r: cpu.keys[0xD] = ev.type==SDL_KEYDOWN; if (cpu.keys[0xD]){tmp=1;} break;
		    case SDLK_a: cpu.keys[0x7] = ev.type==SDL_KEYDOWN; if (cpu.keys[0x7]){tmp=1;} break;
		    case SDLK_s: cpu.keys[0x8] = ev.type==SDL_KEYDOWN; if (cpu.keys[0x8]){tmp=1;} break;
		    case SDLK_d: cpu.keys[0x9] = ev.type==SDL_KEYDOWN; if (cpu.keys[0x9]){tmp=1;} break;
		    case SDLK_f: cpu.keys[0xE] = ev.type==SDL_KEYDOWN; if (cpu.keys[0xE]){tmp=1;} break;
		    case SDLK_z: cpu.keys[0xA] = ev.type==SDL_KEYDOWN; if (cpu.keys[0xA]){tmp=1;} break;
		    case SDLK_x: cpu.keys[0x0] = ev.type==SDL_KEYDOWN; if (cpu.keys[0x0]){tmp=1;} break;
		    case SDLK_c: cpu.keys[0xB] = ev.type==SDL_KEYDOWN; if (cpu.keys[0xB]){tmp=1;} break;
		    case SDLK_v: cpu.keys[0xF] = ev.type==SDL_KEYDOWN; if (cpu.keys[0xF]){tmp=1;} break;
		    case SDLK_5: cpu.keys[0x5] = ev.type==SDL_KEYDOWN; if (cpu.keys[0x5]){tmp=1;} break;
		    case SDLK_6: cpu.keys[0x6] = ev.type==SDL_KEYDOWN; if (cpu.keys[0x6]){tmp=1;} break;
		    case SDLK_7: cpu.keys[0x7] = ev.type==SDL_KEYDOWN; if (cpu.keys[0x7]){tmp=1;} break;
		    case SDLK_8: cpu.keys[0x8] = ev.type==SDL_KEYDOWN; if (cpu.keys[0x8]){tmp=1;} break;
		    case SDLK_9: cpu.keys[0x9] = ev.type==SDL_KEYDOWN; if (cpu.keys[0x9]){tmp=1;} break;
		    case SDLK_0: cpu.keys[0x0] = ev.type==SDL_KEYDOWN; if (cpu.keys[0x0]){tmp=1;} break;
		    case SDLK_ESCAPE: running=0; break;
		    }
		  if(ev.type==SDL_KEYDOWN && (cpu.wait_key & 0x80))
                    {
		      cpu.wait_key        &= 0x7F;
		      cpu.V[cpu.wait_key] = tmp;
                    }
            }
    
       	struct timespec cur_t;
	clock_gettime(CLOCK_MONOTONIC_RAW, &cur_t);
	double elapsed = (( 1000.0*cur_t.tv_sec + 1e-6*cur_t.tv_nsec) - (1000.0*start_t.tv_sec + 1e-6*start_t.tv_nsec))/1000;
        int frames = int(elapsed * 60) - frames_done;
        if(frames > 0)
        {
            frames_done += frames;
            
	    if (cpu.DT > 0) { --cpu.DT; }
	    if (cpu.ST > 0) { printf("Beep!\n\a"); --cpu.ST; }
            
	    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
	    SDL_RenderClear(renderer);
	    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
	    	    
	    SDL_Rect* destRect = (SDL_Rect *)malloc(sizeof(SDL_Rect));
	    destRect->x = 0;
	    destRect->y = 0;
	    destRect->w = 8;
	    destRect->h = 8;
	    int x,y;  
	    for (y = 0; y < H; y++)
	      for (x = 0; x < W; x++)
		if (cpu.gfx[(y * 64) + x] == 1) {
			destRect->x = x * 8;
			destRect->y = y * 8;

			SDL_RenderFillRect(renderer, destRect);
		}
	    	    
	    free (destRect);
	    SDL_RenderPresent(renderer);
        }
        max_consecutive_insns = fmax(frames, 1) * insns_per_frame;
        if((cpu.wait_key & 0x80) || !frames) SDL_Delay(1000/60);
    }
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  
  return 0;
}
