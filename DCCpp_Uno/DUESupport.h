/**********************************************************************

  DUESupport.h
  Additional code for Arduino DUE target

  includes:

  - Timer setup for the relevant Due timers
  - Support for external SPI FRAM (no EEPROM available with the DUE board)
  
**********************************************************************/

#include <SPI.h>            //  SPI support for storing data in external FRAM


#ifndef DUESupport_h
#define DUESupport_h

/////////////////////////////////////////////////////////////////////////////////////
// DEFINE DUE FRAM STORAGE OPTIONS
/////////////////////////////////////////////////////////////////////////////////////

#define FRAM_SIZE           0x1fff    //  for the case that the FRAM size can't be read out from the chip

#define FRAM_CS_PIN         52        //  defines the pin for the FRAM chip select (CS) signal; for hardware SPI: pins 4,10 and 52
#define FRAM_CLK_DIV        4.2       //  defines SPI clock (84 MHz : 4.2 = 20 MHz); there are higher frequency FRAM types available, look for type-specific FRAM clock limits

/////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////
// ARDUINO DUE TIMER REGISTER ADDRESSES
/////////////////////////////////////////////////////////////////////////////////////
  
  #define TC0RA *(volatile uint32_t *) 0x40080014      //  TIOB0   Pin 13 (LED), TIOB0 timer signal
  #define TC0RB *(volatile uint32_t *) 0x40080018      //  TIOB0   values assignable with TC00_RB = ...
  #define TC0RC *(volatile uint32_t *) 0x4008001C      //  TIOB0
  #define TC8RA *(volatile uint32_t *) 0x40088094      //  TIOB8   Pin 12, TIOB8 timer signal
  #define TC8RB *(volatile uint32_t *) 0x40088098      //  TIOB8
  #define TC8RC *(volatile uint32_t *) 0x4008809C      //  TIOB8

  #define TC0SR *(volatile uint32_t *) 0x40080020      //  TIOB0   timer status registers, read-only. Reading clears 
  #define TC8SR *(volatile uint32_t *) 0x400880A0      //  TIOB8   some TC status flags. accessed inside ISR (TC handler) 


///////////////////////////////////////////////////////////////////////////////
// OP CODES FOR SPI FRAM ACCESS
///////////////////////////////////////////////////////////////////////////////

  #define OP_WREN    0b00000110      //  Write enable
  #define OP_WRITE   0b00000010      //  Write
  #define OP_READ    0b00000011      //  Read
  #define OP_RDSR    0b00000101      //  Read status register
  #define OP_WRSR    0b00000001      //  Write status register     
  #define OP_WRDI    0b00000100      //  Write disable
  #define OP_RDID    0b10011111      //  Read device ID (9 bytes)

  //  The following op codes may not be supported by all SPI devices
  
  #define OP_SLEEP   0b10111001      //  Sleep mode
  #define OP_FSTRD   0b00001011      //  Fast read, reads data from FRAM array (at 40 MHz?)
  #define OP_SNR     0b11000011      //  Read serial number (8 bytes)  


///////////////////////////////////////////////////////////////////////////////
// ARDUINO DUE TIMER SETUP
///////////////////////////////////////////////////////////////////////////////

static inline void prepareTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t cl_ticks, uint8_t prescale)
{

  uint8_t TC_CMR_TCCLKS_TIMER_PRESCALE = log(prescale/2)/log(4);  //  This may look a bit mathematical but it simply transforms the prescale factor {2,8,32,128} into
                                                                  //  a value of {0,1,2,3} for setting the relevant bits in the TC channel mode (CMR) register.
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);

  TC_Configure(tc, channel, TC_CMR_WAVE   // Waveform mode ("PWM")            
     | TC_CMR_WAVSEL_UP_RC                // Counts up with automatic trigger on RC compare
     | TC_CMR_TCCLKS_TIMER_PRESCALE       // Set the prescale factor, CMR bits 0 to 2, (MCLK/n) with n = {2,8,32,128}
     | TC_CMR_BCPB_CLEAR                  // Switch TIOB off when reaching RB Compare
     | TC_CMR_BCPC_SET                    // Switch TIOB on  when reaching RC Compare 
     | TC_CMR_EEVT_XC0);                  // Set external events from XC0 (setup TIOB as output)

  TC_SetRA(tc, channel, cl_ticks/2);      //50% high, 50% low
  TC_SetRB(tc, channel, cl_ticks/2);      //50% high, 50% low
  TC_SetRC(tc, channel, cl_ticks);
  
  TC_Start(tc, channel);
  
  tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;     //  Interrupt on RC compare  
  tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS; 
  
//  NVIC_EnableIRQ(irq);                                        //  this is done as last step in setup()
}


///////////////////////////////////////////////////////////////////////////////
// SPI FRAM support
///////////////////////////////////////////////////////////////////////////////

struct SPI_FRAM_Class
{
  private:
        
  public:

    void writeEnable()
    {
      digitalWrite(FRAM_CS_PIN, LOW);
      SPI.transfer(FRAM_CS_PIN, OP_WREN);                  //  write enable
      digitalWrite(FRAM_CS_PIN, HIGH);
    }

    void writeDisable()
    {
      digitalWrite(FRAM_CS_PIN, LOW);
      SPI.transfer(FRAM_CS_PIN,OP_WRDI);                   //  write disable
      digitalWrite(FRAM_CS_PIN, HIGH);
    }
    
    void write(uint32_t index, uint8_t data)
    {
      if (index > FRAM_SIZE) return;                       //  check for memory overflow
      
      digitalWrite(FRAM_CS_PIN, LOW);
      SPI.transfer(FRAM_CS_PIN, OP_WREN);                  //  write enable
      SPI.transfer(FRAM_CS_PIN, OP_WRITE, SPI_CONTINUE);
      SPI.transfer(FRAM_CS_PIN, (uint8_t)(index >> 8), SPI_CONTINUE);
      SPI.transfer(FRAM_CS_PIN, (uint8_t)(index), SPI_CONTINUE);
      SPI.transfer(FRAM_CS_PIN, data);
 
      SPI.transfer(FRAM_CS_PIN, OP_WRDI);                  //  write disable
      digitalWrite(FRAM_CS_PIN, HIGH);
         
      //  Serial.print("FRAM write: addr: ");
      //  Serial.print(index);
      //  Serial.print(" ");
      //  Serial.println(data);
    }

    uint8_t read(uint32_t index)
    {
      byte data = 0xFF;

      if (index > FRAM_SIZE) return 0;                     //  check for memory overflow
      
      digitalWrite(FRAM_CS_PIN, LOW);
      SPI.transfer(FRAM_CS_PIN, OP_READ, SPI_CONTINUE);
      SPI.transfer(FRAM_CS_PIN, index >> 8, SPI_CONTINUE);
      SPI.transfer(FRAM_CS_PIN, index, SPI_CONTINUE);
      data = SPI.transfer(FRAM_CS_PIN, 0);
      digitalWrite(FRAM_CS_PIN, HIGH);

      //  Serial.print("FRAM read: addr: ");
      //  Serial.print(index);
      //  Serial.print(" ");
      //  Serial.println(data);
              
      return data;
    }
    
uint32_t getFRAMSize()                //  returns last FRAM address, see description of RDID command in
{                                     //  Fujitsu MB85RS64V datasheet, page 10. May not work for other chips.
  uint8_t val = 0;
  
  digitalWrite(FRAM_CS_PIN, LOW);
  SPI.transfer(FRAM_CS_PIN, OP_RDID, SPI_CONTINUE);      //  read device ID
  val = SPI.transfer(FRAM_CS_PIN, 0, SPI_CONTINUE);      //  manufacturer ID
  val = SPI.transfer(FRAM_CS_PIN, 0, SPI_CONTINUE);      //  continuation code
  val = SPI.transfer(FRAM_CS_PIN, 0);                    //  product ID (1st byte)
  digitalWrite(FRAM_CS_PIN, HIGH);

  val &= 0x1F;                        //  extract density val (bits 0..4)

  if (val == 0) return val;

  uint32_t mem = (0x400 << val) - 1;  //  convert to last FRAM address

  return mem;
}


 
template <class T> int put(uint32_t index, const T& data)   //  write object
{
    const uint8_t* ptr = (const byte*)(const void*)&data;
    uint8_t i;
    for (i = 0; i < sizeof(data); i++)
          write(index++, *ptr++);
    return i;
}

template <class T> int get(uint32_t index, T& data)         //  read object
{
    uint8_t* ptr = (byte*)(void*)&data;
    uint8_t i;
    for (i = 0; i < sizeof(data); i++)
          *ptr++ = read(index++);
    return i;
}

};

static SPI_FRAM_Class EEPROM;   // named EEPROM for compatibility
//#endif

#endif
