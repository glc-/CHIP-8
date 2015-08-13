#include <stdlib.h> /* for rand() */
#include <SDL.h>
#include <fstream>
#include <unistd.h>

#define W 64
#define H 32

unsigned char mem[0x1000];
unsigned char V[16];
unsigned short I, PC, SP, stack[16];
unsigned char DT, ST; /* Delay Timer, Sound Timer */

unsigned short opcode;

unsigned char gfx[W * H];
unsigned char draw;
unsigned char keys[16];
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


void font ()
{
  int i;
  for (i=0; i<80; i++)
      mem[i] = chip8_fontset[i];
}

void reset_keys (unsigned char** keys) {
  int i;
  for (i=0; i<16; i++)
    keys[i] = 0;
}

void load(const char* filename, unsigned pos = 0x200)
{
  for(std::ifstream f(filename, std::ios::binary); f.good(); )
    mem[pos++ & 0xFFF] = f.get();
}

void exec ()
{
  opcode = (mem[PC] << 8) | (mem[PC+1]);
  PC +=2;
  
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
		  gfx[i] = 0; 
		}
	    }
	    break;
 
	  case 0x00ee:
	    {
	      PC = stack[SP--];
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
	PC = (opcode & 0x0fff); 
	
      }
      break;

    case 0x2000: 
      { 
	stack[++SP] = PC;
	PC = (opcode & 0x0fff);
      }
      break;

    case 0x3000:
      {
	if (V[(opcode & 0x0f00) >> 8] == (opcode & 0x00ff))
	    PC+=2;
      }
      break;

    case 0x4000:
      {
	if (V[(opcode & 0x0f00) >> 8] != (opcode & 0x00ff))
	    PC+=2;
      }
      break;

    case 0x5000: 
      {
	if (V[(opcode & 0x0f00) >> 8] == V[(opcode & 0x00f0) >> 4])
	  {
	    PC+=2;
	  }
      }
      break;

    case 0x6000:
      {
	V[(opcode & 0x0f00) >> 8] = (opcode & 0x00ff); 
      }
      break;

    case 0x7000:
      {
	V[(opcode & 0x0f00) >> 8] = V[(opcode & 0x0f00) >> 8] + (opcode & 0x00ff); 
      }
      break;

    case 0x8000: 
      {
	switch (opcode & 0x000f) 
	  {
	  case 0x0000:
	    {
	      V[(opcode & 0x0f00) >> 8] = V[(opcode & 0x00f0) >> 4]; 
	    } 
	    break;

	  case 0x0001:
	    { 
	      V[(opcode & 0x0f00) >> 8] = V[(opcode & 0x0f00) >> 8] | V[(opcode & 0x00f0) >> 4];
	    }
	    break;

	  case 0x0002:
	    {
	      V[(opcode & 0x0f00) >> 8] = V[(opcode & 0x0f00) >> 8] & V[(opcode & 0x00f0) >> 4]; 
	    }
	    break;

	  case 0x0003:
	    {
	      V[(opcode & 0x0f00) >> 8] = V[(opcode & 0x0f00) >> 8] ^ V[(opcode & 0x00f0) >> 4];
	    }
	    break;

	  case 0x0004:
	    {
	      unsigned short tmp = V[(opcode & 0x0f00) >> 8] + V[(opcode & 0x00f0) >> 4];
	      if ( tmp > 255 ) {
		V[0xf] = 1;
	      } else {
		V[0xf] = 0;
	      }
	      V[(opcode & 0x0f00) >> 8] = (tmp & 0x00ff);
	    }
	    /*{
	      unsigned short p = V[(opcode & 0x0F00) >> 8] + V[(opcode & 0x00F0) >> 4];
	      V[0xf] = (p>>8);
	      V[(opcode & 0x0F00) >> 8] = p;
	      }*/
	    break;   
  
	  case 0x0005:
	    {
	      if ( V[(opcode & 0x0f00) >> 8] > V[(opcode & 0x00f0) >> 4] ) {
		V[0xf] = 1;
	      } else {
		V[0xf] = 0;
	      }
	      V[(opcode & 0x0f00) >> 8] -= V[(opcode & 0x00f0) >> 4];
	    }
	    break;

	  case 0x0006:
	    {
	      V[0xf] = ( V[(opcode & 0x0f00) >> 8] & 1 );
	      V[(opcode & 0x0f00) >> 8] >>= 1;
	    }
	    break;

	  case 0x0007:
	    {
	      if ( V[(opcode & 0x00f0) >> 4] > V[(opcode & 0x0f00) >> 8] ) {
		V[0xf] = 1;
	      } else {
		V[0xf] = 0;
	      }
	      V[(opcode & 0x0f00) >> 8] = V[(opcode & 0x00f0) >> 4] - V[(opcode & 0x0f00) >> 8];
	    }
	    break;

	  case 0x000e:
	    {
	      if ( V[(opcode & 0x0f00) >> 8] > 7 ) {
		V[0xf] = 1;
	      } else {
		V[0xf] = 0;
	      }
	      /*
	      V[0xF] = V[(opcode & 0x0F00) >> 8] >> 7;
	      V[(opcode & 0x0f00) >> 8] <<= 1;
	      */
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
	if (V[(opcode & 0x0f00) >> 8] != V[(opcode & 0x00f0) >> 4])
	  {
	    PC+=2;
	  } 
      }

      break;
    case 0xa000:
      {
	I = (opcode & 0x0fff);
      }
      break;

    case 0xb000:
      {
	PC = (opcode & 0x0fff) + V[0]; 
      }
      break;

    case 0xc000:
      {
	V[(opcode & 0x0f00) >> 8] = (rand() % 255) & (opcode & 0x00ff);
      }
      break;

    case 0xd000:
      {
	unsigned short x = V[(opcode & 0x0F00) >> 8];
	unsigned short y = V[(opcode & 0x00F0) >> 4];
	unsigned short height = opcode & 0x000F;
	unsigned short pixel;
     
	int xline, yline;
	  
	V[0xF] = 0;
	for (yline = 0; yline < height; yline++)
	  {
	    pixel = mem[I + yline];
	    for(xline = 0; xline < 8; xline++)
	      {
		if((pixel & (0x80 >> xline)) != 0)
		  {
		    if(gfx[(x + xline + ((y + yline) * 64))] == 1)
		      V[0xF] = 1;                                 
		    gfx[x + xline + ((y + yline) * 64)] ^= 1;
		  }
	      }
	  }
	draw = 1;
      }
      break;

      /****************************************************************
Ex9E - SKP Vx
Skip next instruction if key with the value of Vx is pressed.

Checks the keyboard, and if the key corresponding to the value of Vx is currently in the down position, PC is increased by 2.
      ****************************************************************/

    case 0xe000:
      {
	switch (opcode & 0x000f)
	  {
	  case 0x000e: { } 
	    break;


      /****************************************************************
ExA1 - SKNP Vx
Skip next instruction if key with the value of Vx is not pressed.

Checks the keyboard, and if the key corresponding to the value of Vx is currently in the up position, PC is increased by 2.
      ****************************************************************/

	  case 0x0001: { PC +=2; }
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
	      V[(opcode & 0x0f00) >> 8] = DT;
	    }
	    break;

	    /****************************************************************
Fx0A - LD Vx, K
Wait for a key press, store the value of the key in Vx.

All execution stops until a key is pressed, then the value of that key is stored in Vx.
	    ****************************************************************/

	  case 0x000a: { } 
	    break;
 
	  case 0x0015:
	    {
	      DT = V[(opcode & 0x0f00) >> 8];
	    }
	    break;

	  case 0x0018:
	    {
	      ST = V[(opcode & 0x0f00) >> 8];
	    }
	    break;

	  case 0x001e:
	    {
	      I += V[(opcode & 0x0f00) >> 8];
	    }
	    break;

	  case 0x0029:
	    {
	      I = V[(opcode & 0x0f00) >> 8] * 5;
	    }
	    break; /* Fonts start at address 0x0000 and occupy 5 bytes, so *5 is for the offset. */

	  case 0x0033:
	    { 
	      mem[I] = V[(opcode & 0x0f00) >> 8] / 100;
	      mem[I+1] = ( V[(opcode & 0x0f00) >> 8] % 100 ) / 10;
	      mem[I+2] = V[(opcode & 0x0f00) >> 8] % 10;
	    }
	    break;

	  case 0x0055:
	    {
	      int index;
	      for (index=0; index <= ( (opcode & 0x0f00) >> 8 ); index++) {
		mem[I + index] = V[index];
	      }
	    }
	    break;

	  case 0x0065:
	    {
	      int index;
	      for (index=0; index <= ( (opcode & 0x0f00) >> 8 ); index++) {
		V[index] =  mem[I + index];
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

  if (DT > 0) { --DT; }
  if (ST > 0) { printf("Beep!\n\a"); --ST; }

  SDL_Delay(1000/60);
} 



int main (int argc, char** argv)
{
  SP = 0;
  PC = 0x200;
  font();
  load(argv[1]);

  /* Create a screen. */

  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window *window;
  window = SDL_CreateWindow("Chip-8", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 512, 256, NULL);
  SDL_Renderer *renderer;
  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
  SDL_Event events;
  SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
  SDL_RenderClear(renderer);

  int running=1;
  while(running)
    {
      	  // Clear screen
	  SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
	  SDL_RenderClear(renderer);

	  while (SDL_PollEvent(&events))
	    {
	      switch (events.type)
		{
		  // Check to see if X on window was hit
		case SDL_QUIT:
		  {
		    running = 0;
		  } break;
		}
	    }

	  exec();



	  if(draw) {
	  // Clear screen
	  SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
	  SDL_RenderClear(renderer);
	  // Draw screen
	  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
	  SDL_Rect *destRect = new SDL_Rect;
	  destRect->x = 0;
	  destRect->y = 0;
	  destRect->w = 8;
	  destRect->h = 8;

	  for (int y = 0; y < 32; y++)
	    {
	      for (int x = 0; x < 64; x++)
		{
		  if (gfx[(y * 64) + x] == 1)
		    {
		      destRect->x = x * 8;
		      destRect->y = y * 8;

		      SDL_RenderFillRect(renderer, destRect);
		    }
		}
	    }

	  delete destRect;

	  SDL_RenderPresent(renderer);
	  draw=0;
	  }
    }

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
	
  return 0;
}
