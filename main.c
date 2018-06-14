/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.65.2
        Device            :  PIC16LF1459
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include <xc.h>
#include "mcc_generated_files/mcc.h"
#include "rfid.h"

/*
int MFRC522_CS;
int MFRC522_Rst;
int MFRC522_CS_Direction;
int MFRC522_Rst_Direction;
 */

#define MFRC522_Rst LATCbits.LATC4 
/*
#define MFRC522_CS  LATCbits.LATC6      
 

#define MFRC522_CS_Direction   TRISCbits.TRISC6;
#define MFRC522_Rst_Direction  TRISCbits.TRISC4;
*/


uint8_t     readDummy;

static void MFRC522_Wr( char addr, char value )
{
        //MFRC522_CS = 0;
        //CSN = 0;
        IO_RC6_SetLow();
        SPI_Exchange8bit( ( addr << 1 ) & 0x7E );
        readDummy = SPI_Exchange8bit( value );
        //MFRC522_CS = 1;
        IO_RC6_SetHigh();
        //CSN = 1;
}



static char MFRC522_Rd( char addr )
{
char value;
        //MFRC522_CS = 0;
        //CSN = 0;
        IO_RC6_SetLow();
        SPI_Exchange8bit( (( addr << 1 ) & 0x7E) | 0x80 );
        value = SPI_Exchange8bit( 0x00 );
        //MFRC522_CS = 1;
        //CSN = 1;
        IO_RC6_SetHigh();
        return value;
}



static void MFRC522_Clear_Bit( char addr, char mask )
{
     MFRC522_Wr( addr, MFRC522_Rd( addr ) & (~mask) );
}


static void MFRC522_Set_Bit( char addr, char mask )
{
     MFRC522_Wr( addr, MFRC522_Rd( addr ) | mask );
}

void MFRC522_Reset()
{
        MFRC522_Wr( COMMANDREG, PCD_RESETPHASE );
}



void MFRC522_AntennaOn()
{
 MFRC522_Set_Bit( TXCONTROLREG, 0x03 );
}



void MFRC522_AntennaOff()
{
 MFRC522_Clear_Bit( TXCONTROLREG, 0x03 );
}



void MFRC522_Init()
{
     //MFRC522_CS = 1;
     IO_RC6_SetHigh();
     MFRC522_Rst = 1;
              
     //MFRC522_CS_Direction = 0;
     //MFRC522_Rst_Direction = 0;
     
        
     MFRC522_Reset();
        
     MFRC522_Wr( TMODEREG, 0x8D );      //Tauto=1; f(Timer) = 6.78MHz/TPreScaler
     MFRC522_Wr( TPRESCALERREG, 0x3E ); //TModeReg[3..0] + TPrescalerReg
     MFRC522_Wr( TRELOADREGL, 30 );           
     MFRC522_Wr( TRELOADREGH, 0 );
        
     MFRC522_Wr( TXAUTOREG, 0x40 );    //100%ASK
     MFRC522_Wr( MODEREG, 0x3D );      // CRC valor inicial de 0x6363
        
     //MFRC522_Clear_Bit( STATUS2REG, 0x08 );//MFCrypto1On=0
     //MFRC522_Wr( RXSELREG, 0x86 );      //RxWait = RxSelReg[5..0]
     //MFRC522_Wr( RFCFGREG, 0x7F );     //RxGain = 48dB

     MFRC522_AntennaOn();
}



char MFRC522_ToCard( char command, char *sendData, char sendLen, char *backData, unsigned *backLen )
{
  char _status = MI_ERR;
  char irqEn = 0x00;
  char waitIRq = 0x00;
  char lastBits;
  char n;
  unsigned i;
  
  switch (command)
  {
    case PCD_AUTHENT:       //Certification cards close
    {
      irqEn = 0x12;
      waitIRq = 0x10;
      break;
    }
    case PCD_TRANSCEIVE:    //Transmit FIFO data
    {
      irqEn = 0x77;
      waitIRq = 0x30;
      break;
    }
    default:
      break;
  }

  MFRC522_Wr( COMMIENREG, irqEn | 0x80 );  //Interrupt request
  MFRC522_Clear_Bit( COMMIRQREG, 0x80 );   //Clear all interrupt request bit
  MFRC522_Set_Bit( FIFOLEVELREG, 0x80 );   //FlushBuffer=1, FIFO Initialization
  MFRC522_Wr( COMMANDREG, PCD_IDLE );      //NO action; Cancel the current command???
  
  
  
  //Writing data to the FIFO
  for ( i=0; i < sendLen; i++ )
  {
    MFRC522_Wr( FIFODATAREG, sendData[i] );
  }

  //Execute the command
  MFRC522_Wr( COMMANDREG, command );

  if (command == PCD_TRANSCEIVE )
  {
    MFRC522_Set_Bit( BITFRAMINGREG, 0x80 ); //StartSend=1,transmission of data starts  
  }

  //Waiting to receive data to complete
  //i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms???
  i = 0xFFFF;   
  do
  {
    //CommIrqReg[7..0]
    //Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
    n = MFRC522_Rd( COMMIRQREG );
    i--;
  }
  while ( i && !(n & 0x01) && !( n & waitIRq ) );

  MFRC522_Clear_Bit( BITFRAMINGREG, 0x80 );    //StartSend=0

  if (i != 0)
  {
    if( !( MFRC522_Rd( ERRORREG ) & 0x1B ) ) //BufferOvfl Collerr CRCErr ProtecolErr
    {
      _status = MI_OK;
      if ( n & irqEn & 0x01 )
      {
        _status = MI_NOTAGERR;       //??
      }

      if ( command == PCD_TRANSCEIVE )
      {
        n = MFRC522_Rd( FIFOLEVELREG );
        lastBits = MFRC522_Rd( CONTROLREG ) & 0x07;
        if (lastBits)
        {
          *backLen = (n-1) * 8 + lastBits;
        }
        else
        {
          *backLen = n * 8;
        }

        if (n == 0)
        {
          n = 1;
        }

        if (n > 16)
        {
          n = 16;
        }

        //Reading the received data in FIFO
        for (i=0; i < n; i++)
        {
          backData[i] = MFRC522_Rd( FIFODATAREG );
        }
  
  backData[i] = 0;
      }
    }
    else
    {
      _status = MI_ERR;
    }

  }

  //MFRC522_Set_Bit( CONTROLREG, 0x80 );
  //MFRC522_Wr( COMMANDREG, PCD_IDLE );

  return _status;
}



char MFRC522_Request( char reqMode, char *TagType )
{
  char _status;
  unsigned backBits;            //The received data bits

  MFRC522_Wr( BITFRAMINGREG, 0x07 ); //TxLastBists = BitFramingReg[2..0]   ???

  TagType[0] = reqMode;
  _status = MFRC522_ToCard( PCD_TRANSCEIVE, TagType, 1, TagType, &backBits );

  if ( (_status != MI_OK) || (backBits != 0x10) )
  {
    _status = MI_ERR;
  }

  return _status;
}



void MFRC522_CRC( char *dataIn, char length, char *dataOut )
{
char i, n;
    MFRC522_Clear_Bit( DIVIRQREG, 0x04 );
    MFRC522_Set_Bit( FIFOLEVELREG, 0x80 );    
    
 //Escreve dados no FIFO        
    for ( i = 0; i < length; i++ )
    {   
        MFRC522_Wr( FIFODATAREG, *dataIn++ );   
    }
    
    MFRC522_Wr( COMMANDREG, PCD_CALCCRC );
        
    i = 0xFF;
    //Espera a finalização do Calculo do CRC
    do 
    {
        n = MFRC522_Rd( DIVIRQREG );
        i--;
    }
    while( i && !(n & 0x04) );        //CRCIrq = 1
        
    dataOut[0] = MFRC522_Rd( CRCRESULTREGL );
    dataOut[1] = MFRC522_Rd( CRCRESULTREGM );        
}



char MFRC522_SelectTag( char *serNum )
{
  char i;
  char _status;
  char size;
  unsigned recvBits;
  char buffer[9];
  
  //MFRC522_Clear_Bit( STATUS2REG, 0x08 );   //MFCrypto1On=0
  
  buffer[0] = PICC_SElECTTAG;
  buffer[1] = 0x70;
  
  for ( i=2; i < 7; i++ )
  {
    buffer[i] = *serNum++;
  }
  
  MFRC522_CRC( buffer, 7, &buffer[7] );             
  
  _status = MFRC522_ToCard( PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits );

  if ( (_status == MI_OK) && (recvBits == 0x18) )
  {
    size = buffer[0];
  }
  else
  {
    size = 0;
  }

  return size;
}



//hibernation
void MFRC522_Halt()
{
  unsigned unLen;
  char buff[4];
  
  buff[0] = PICC_HALT;
  buff[1] = 0;
  MFRC522_CRC( buff, 2, &buff[2] );

  MFRC522_Clear_Bit( STATUS2REG, 0x80 );
  MFRC522_ToCard( PCD_TRANSCEIVE, buff, 4, buff, &unLen );
  MFRC522_Clear_Bit( STATUS2REG, 0x08 );
}



char MFRC522_Auth( char authMode, char BlockAddr, char *Sectorkey, char *serNum )
{
  char _status;
  unsigned recvBits;
  char i;
  char buff[12];
  
  //Verify the command block address + sector + password + card serial number
  buff[0] = authMode;
  buff[1] = BlockAddr;
  
  for ( i = 2; i < 8; i++ )
  {
    buff[i] = Sectorkey[i-2];
  }
  
  for ( i = 8; i < 12; i++ )
  {
    buff[i] = serNum[i-8];
  }
  
  _status = MFRC522_ToCard( PCD_AUTHENT, buff, 12, buff, &recvBits );
  
  if ( ( _status != MI_OK ) || !( MFRC522_Rd( STATUS2REG ) & 0x08 ) )
  {
    _status = MI_ERR;
  }
  
  return _status;
}



char MFRC522_Write( char blockAddr, char *writeData )
{
  char _status;
  unsigned recvBits;
  char i;
  char buff[18];

  buff[0] = PICC_WRITE;
  buff[1] = blockAddr;
  
  MFRC522_CRC( buff, 2, &buff[2] );
  _status = MFRC522_ToCard( PCD_TRANSCEIVE, buff, 4, buff, &recvBits );

  if ( (_status != MI_OK) || (recvBits != 4) || ( (buff[0] & 0x0F) != 0x0A) )
  {
    _status = MI_ERR;
  }

  if (_status == MI_OK)
  {
    for ( i = 0; i < 16; i++ )                //Data to the FIFO write 16Byte
    {
      buff[i] = writeData[i];
    }
    
    MFRC522_CRC( buff, 16, &buff[16] );
    _status = MFRC522_ToCard( PCD_TRANSCEIVE, buff, 18, buff, &recvBits );

    if ( (_status != MI_OK) || (recvBits != 4) || ( (buff[0] & 0x0F) != 0x0A ) )
    {
      _status = MI_ERR;
    }
  }

  return _status;
}



char MFRC522_Read( char blockAddr, char *recvData )
{
  char _status;
  unsigned unLen;

  recvData[0] = PICC_READ;
  recvData[1] = blockAddr;
  
  MFRC522_CRC( recvData, 2, &recvData[2] );
  
  _status = MFRC522_ToCard( PCD_TRANSCEIVE, recvData, 4, recvData, &unLen );

  if ( (_status != MI_OK) || (unLen != 0x90) )
  {
    _status = MI_ERR;
  }

  return _status;
}



char MFRC522_AntiColl( char *serNum )
{
  char _status;
  char i;
  char serNumCheck = 0;
  unsigned unLen;

  MFRC522_Wr( BITFRAMINGREG, 0x00 );                //TxLastBists = BitFramingReg[2..0]

  serNum[0] = PICC_ANTICOLL;
  serNum[1] = 0x20;
  MFRC522_Clear_Bit( STATUS2REG, 0x08 );
  _status = MFRC522_ToCard( PCD_TRANSCEIVE, serNum, 2, serNum, &unLen );

  if (_status == MI_OK)
  {
    for ( i=0; i < 4; i++ )
    {
      serNumCheck ^= serNum[i];
    }
    
    if ( serNumCheck != serNum[4] )
    {
      _status = MI_ERR;
    }
  }

  return _status;
}


//0x0044 = Mifare_UltraLight
//0x0004 = Mifare_One (S50)
//0x0002 = Mifare_One (S70)
//0x0008 = Mifare_Pro (X)
//0x0344 = Mifare_DESFire
 
char MFRC522_isCard( char *TagType ) 
{
    if (MFRC522_Request( PICC_REQIDL, TagType ) == MI_OK)
        return 1;
    else
        return 0; 
}



char MFRC522_ReadCardSerial( char *str )
{
    char _status; 
    _status = MFRC522_AntiColl( str );
    str[5] = 0;
     if (_status == MI_OK)
      return 1;
     else
      return 0;
}


/*
                         Main application
 */
void main(void)
{
    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    //INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    
    PIN_MANAGER_Initialize();
    
    TRISA = 0xFF;
    TRISB = 0xFF;
    TRISC = 0x00;
    
    REDLED = 0;
    GREENLED = 0;

    //Declarations
      char msg[12];
      char UID[6];
      char key[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
      char writeData[] = "1234567890123456";
      unsigned TagType;
      char size;
      char i;
      
      
      //Desativa as Portas Analogicas
      //ADCON1 = 0x0F;

      //Inicializa Soft SPI
      //Soft_SPI_Init(); 
      SPI_Initialize();

      //inicializa o modulo RFID
      MFRC522_Init();

      while(1)
      {
       //Verifica se há algum cartão
       if( MFRC522_isCard( &TagType ) )
       {
           REDLED = 1;
           __delay_ms(50);
       }
       
       

           //Faz a leitura do numero de serie
           if( MFRC522_ReadCardSerial( &UID ) )
           {
               size = MFRC522_SelectTag( &UID );
           }

           //Tenta realizar a autenticação A do setor 1( blocos: 4 - 7 )
           //bloco de autenticação é o 7

           if( MFRC522_Auth( PICC_AUTHENT1A, 7, &key, &UID ) == 0 )
           {
               //Escreve algo no bloco 4
               MFRC522_Write( 4, &writeData );
           }
           else if( MFRC522_Auth( PICC_AUTHENT1B, 7, &key, &UID ) == 0 )
           {
               //Escreve algo no bloco 4
               MFRC522_Write( 4, &writeData );
           }
           else
           {
              continue;
           }

           //Faz a leitura do bloco 4
           if( MFRC522_Read( 4, &writeData ) == 0 )
           {
           }

           //Estado de hibernação
           //MFRC522_Halt();
      }
}
/**
 End of File
*/