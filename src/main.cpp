/*
 * This file is part of the stm32-template project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/iwdg.h>
#include "stm32_can.h"
#include "canmap.h"
#include "cansdo.h"
#include "terminal.h"
#include "params.h"
#include "hwdefs.h"
#include "digio.h"
#include "hwinit.h"
#include "anain.h"
#include "linbus.h"
#include "param_save.h"
#include "my_math.h"
#include "errormessage.h"
#include "printf.h"
#include "stm32scheduler.h"
#include "terminalcommands.h"

#define PRINT_JSON 0

static Stm32Scheduler* scheduler;
static CanHardware* can;
static CanMap* canMap;
static LinBus* lin;
static bool read = true;

static void SendLin()
{
   uint8_t id, len;
   
   //static bool read = true;
   
	//DigIo::lin_cs.Set(); // enable lin transceiver, Bluepill etc
	DigIo::lin_nslp.Set(); // enable lin transceiver

   if (lin->HasReceived(Param::GetInt(Param::linrxid), 8, Param::GetInt(Param::classicChksum)))
   {
      uint8_t* data = lin->GetReceivedBytes();

      Param::SetInt(Param::lindata0, data[0]);
	  Param::SetInt(Param::lindata1, data[1]);
	  Param::SetInt(Param::lindata2, data[2]);
	  Param::SetInt(Param::lindata3, data[3]);
	  Param::SetInt(Param::lindata4, data[4]);
	  Param::SetInt(Param::lindata5, data[5]);
	  Param::SetInt(Param::lindata6, data[6]);
	  Param::SetInt(Param::lindata7, data[7]);
	  Param::SetInt(Param::linavail, 1);
   }

   if (read)
   {
   lin->Request(Param::GetInt(Param::linrxid), 0, 0, Param::GetInt(Param::classicChksum)); // true = classic chksum
   Param::SetInt(Param::linavail, 0);
   }
   else
   {
      uint8_t lindata[8];

      lindata[0] = Param::GetInt(Param::linsend0);
      lindata[1] = Param::GetInt(Param::linsend1);
	  lindata[2] = Param::GetInt(Param::linsend2);
	  lindata[3] = Param::GetInt(Param::linsend3);
 	  lindata[4] = Param::GetInt(Param::linsend4);
	  lindata[5] = Param::GetInt(Param::linsend5);
	  lindata[6] = Param::GetInt(Param::linsend6);
	  lindata[7] = Param::GetInt(Param::linsend7);

   id = Param::GetInt(Param::lintxid);
   len = Param::GetInt(Param::lindatalen);
   
   lin->Request(id, lindata, len, Param::GetInt(Param::classicChksum));
}

   read = !read;
}
uint8_t linrxid = 0;
static void Ms200Task(void)
{
   DigIo::led_rd.Toggle();
   //DigIo::led_out2.Toggle(); // Nucleo F103
   //DigIo::led_out3.Toggle(); // Bluepill
   DigIo::led_bl.Toggle();

   iwdg_reset();
   //Calculate CPU load. Don't be surprised if it is zero.
   float cpuLoad = scheduler->GetCpuLoad();
   //This sets a fixed point value WITHOUT calling the parm_Change() function
   Param::SetFloat(Param::cpuload, cpuLoad / 10);

   if (Param::GetInt(Param::IDsweep))
   { 
     if (lin->HasReceived(linrxid, 2, Param::GetInt(Param::classicChksum))>0) 
     {
         Param::SetInt(Param::IDreturned, linrxid);
     }
     linrxid++;
     if (linrxid >61) linrxid = 0;
     lin->Request(linrxid, 0, 0, Param::GetInt(Param::classicChksum)); // request is after HasReceived to allow time to receive data
   }
   else
   {
       SendLin();
   }
   
   canMap->SendAll();
}

//sample 10 ms task
static void Ms10Task(void)
{
   //Set timestamp of error message
   ErrorMessage::SetTime(rtc_get_counter_val());
}

/** This function is called when the user changes a parameter */
void Param::Change(Param::PARAM_NUM paramNum)
{
   switch (paramNum)
   {  
   case Param::linbaud:
   usart_set_baudrate(USART1, 9600 * (1 + Param::GetInt(Param::linbaud)));
      break;
   default:
      break;
   }
}

//Whichever timer(s) you use for the scheduler, you have to
//implement their ISRs here and call into the respective scheduler
extern "C" void tim2_isr(void)
{
   scheduler->Run();
}

extern "C" int main(void)
{
   extern const TERM_CMD termCmds[];

   clock_setup(); //Must always come first
   rtc_setup();
   //ANA_IN_CONFIGURE(ANA_IN_LIST);
   DIG_IO_CONFIGURE(DIG_IO_LIST);
   //AnaIn::Start(); //Starts background ADC conversion via DMA
   //write_bootloader_pininit(); //Instructs boot loader to initialize certain pins
   gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, 0);//disable JTAG, enable SWD,

   //tim_setup(); //Sample init of a timer
   nvic_setup(); //Set up some interrupts
   parm_load(); //Load stored parameters
   
   LinBus l(USART1, 9600 * (1 + Param::GetInt(Param::linbaud)));
   lin = &l;

   //Initialize CAN1, including interrupts. Clock must be enabled in clock_setup()
   Stm32Can c(CAN1, CanHardware::Baud500, false);
   CanMap cm(&c);
   CanSdo sdo(&c, &cm);
   sdo.SetNodeId(7); //Set node ID for SDO access e.g. by wifi module
   //store a pointer for easier access
   can = &c;
   canMap = &cm;

   //This is all we need to do to set up a terminal on USART3
   Terminal t(USART3, termCmds);
   usart_set_baudrate(USART3, 921600); // overwrite 115200 baud rate in terminal initialisation
   usart_set_stopbits(USART3, USART_STOPBITS_1); // overwrite stopbits
   
   TerminalCommands::SetCanMap(canMap);
   Stm32Scheduler s(TIM2); //We never exit main so it's ok to put it on stack
   scheduler = &s;                                      

   s.AddTask(Ms10Task, 10);
   s.AddTask(Ms200Task, 200);

   //backward compatibility, version 4 was the first to support the "stream" command
   Param::SetInt(Param::version, 4);
   Param::Change(Param::PARAM_LAST); //Call callback one for general parameter propagation


   while(1)
   {
      char c = 0;
      t.Run();
      if (sdo.GetPrintRequest() == PRINT_JSON)
      {
         TerminalCommands::PrintParamsJson(&sdo, &c);
      }
   }

   return 0;
}

