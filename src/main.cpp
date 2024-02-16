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

static void SendLin()
{
   static bool read = true;
   
	DigIo::lin_cs.Set(); // enable lin transceiver, Bluepill etc
	//DigIo::lin_nslp.Set(); // enable lin transceiver, ZV

   if (lin->HasReceived(Param::GetInt(Param::linrxid), 8))
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
   lin->Request(Param::GetInt(Param::linrxid), 0, 0);
   Param::SetInt(Param::linavail, 0);
   }
   else
   {
      uint8_t lindata[4];

      lindata[0] = Param::GetInt(Param::linsend0);
	  lindata[1] = Param::GetInt(Param::linsend1);
	  lindata[2] = Param::GetInt(Param::linsend2);
	  lindata[3] = Param::GetInt(Param::linsend3);

      lin->Request(Param::GetInt(Param::lintxid), lindata, sizeof(lindata));
   }

   read = !read;
}

//sample 100ms task
static void Ms100Task(void)
{

   DigIo::led_out.Toggle(); // ZV
   DigIo::led_out2.Toggle(); // Nucleo F103
   DigIo::led_out3.Toggle(); // Bluepill

   iwdg_reset();
   //Calculate CPU load. Don't be surprised if it is zero.
   float cpuLoad = scheduler->GetCpuLoad();
   //This sets a fixed point value WITHOUT calling the parm_Change() function
   Param::SetFloat(Param::cpuload, cpuLoad / 10);
   
   SendLin();

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
   usart_set_baudrate(USART1, Param::GetInt(Param::linbaud));
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

   clock_setup();
   rtc_setup();
   DIG_IO_CONFIGURE(DIG_IO_LIST);
   gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, 0);
   //write_bootloader_pininit();
   tim_setup();
   nvic_setup();
   parm_load();

   LinBus l(USART1, Param::GetInt(Param::linbaud));
   lin = &l;

   Stm32Can c(CAN1, CanHardware::Baud500, false);
   CanMap cm(&c);
   CanSdo sdo(&c, &cm);
   sdo.SetNodeId(7); //Set node ID for SDO access e.g. by wifi module

   can = &c;
   canMap = &cm;

   Terminal t(USART3, termCmds);
   TerminalCommands::SetCanMap(canMap);
   
   Stm32Scheduler s(TIM2);
   scheduler = &s;

   s.AddTask(Ms10Task, 10);
   s.AddTask(Ms100Task, 100);

   Param::SetInt(Param::version, 4);
   Param::Change(Param::PARAM_LAST);

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

