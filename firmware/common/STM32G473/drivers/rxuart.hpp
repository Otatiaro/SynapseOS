#pragma once

#include <cstdint>
#include <cmath>
#include <cassert>

#include <Callback.hpp>
#include <CortexM.hpp>
#include <Enums.hpp>
#include <IsrPriority.hpp>
#include <PriorityMutex.hpp>

#include <interrupts.hpp>
#include <rcc.hpp>
#include <gpio.hpp>
#include <usart.hpp>
#include <tim.hpp>

extern "C" uint32_t SystemCoreClock;

using namespace STM32G473XX;

namespace neuron2
{
class rxuart
{
public:

	static void init(opsy::IsrPriority priority)
	{
		s_mutex = opsy::PriorityMutex(priority);
		rcc.ahb1enr <<= rcc_p::ahb1enr_r::dma1en_f() || atomic;
		rcc.ahb2enr <<= rcc_p::ahb2enr_r::gpioaen_f() || rcc_p::ahb2enr_r::gpioben_f() || rcc_p::ahb2enr_r::gpiocen_f() || atomic;
		rcc.apb1enr1 <<= rcc_p::apb1enr1_r::usart3en_f() || atomic;
		rcc.apb2enr <<= rcc_p::apb2enr_r::tim16en_f() || atomic;

		gpioa.ospeedr <<= gpioa_p::ospeedr_r::ospeedr15_f(0b00) || atomic;
		gpioa.otyper <<= gpioa_p::otyper_r::ot15_f(false) || atomic;
		gpioa.moder <<= gpioa_p::moder_r::moder15_f(0b00) || atomic;
		gpioa.pupdr <<= gpioa_p::pupdr_r::pupdr15_f(0b00) || atomic;

		usart3.cr1 = usart1_p::cr1_r::ResetValue;
		usart3.cr2 = usart1_p::cr2_r::ResetValue;
		usart3.cr3 = usart1_p::cr3_r::dmat_f() | usart1_p::cr3_r::hdsel_f();
		opsy::CortexM::setIsrHandler(static_cast<uint32_t>(USART3_IRQn), uartIsrHandler);
		opsy::CortexM::setPriority(static_cast<uint32_t>(USART3_IRQn), priority);
		opsy::CortexM::enableInterrupt(static_cast<uint32_t>(USART3_IRQn));
		usart3.cr1 = usart1_p::cr1_r::te_f() | usart1_p::cr1_r::re_f() | usart1_p::cr1_r::ue_f();

		gpioc.ospeedr <<= gpioc_p::ospeedr_r::ospeedr10_f(0b10) || atomic;
		gpioc.otyper <<= gpioc_p::otyper_r::ot10_f() || atomic;
		gpioc.pupdr <<= gpioc_p::pupdr_r::pupdr10_f(0b00) || atomic;
		gpioc.afrh <<= gpioc_p::afrh_r::afrh10_f(7) || atomic;
		gpioc.moder <<= gpioc_p::moder_r::moder10_f(0b10) || atomic;


		tim16.cr1 = tim16_p::cr1_r::ResetValue;
		tim16.cr2 = tim16_p::cr2_r::ResetValue;
		tim16.psc = (SystemCoreClock / TargetFrequency) - 1;
		s_tickTime = static_cast<float>(tim16.psc.load().value() + 1) / static_cast<float>(SystemCoreClock);
		tim16.ccmr1_input = tim16_p::ccmr1_input_r::ic1f_f(0b0011) | tim16_p::ccmr1_input_r::cc1s_f(0b01);
		tim16.arr = tim16_p::arr_r::arr_f(0xFFFF);
		opsy::CortexM::setIsrHandler(static_cast<uint32_t>(TIM1_UP_TIM16_IRQn), timIsrHandler);
		opsy::CortexM::setPriority(static_cast<uint32_t>(TIM1_UP_TIM16_IRQn), priority);
		opsy::CortexM::enableInterrupt(static_cast<uint32_t>(TIM1_UP_TIM16_IRQn));
		tim16.cr1 = tim16_p::cr1_r::cen_f();

		gpiob.pupdr <<= gpiob_p::pupdr_r::pupdr4_f(0b00) || atomic;
		gpiob.afrl <<= gpiob_p::afrl_r::afrl4_f(1) || atomic;
		gpiob.moder <<= gpiob_p::moder_r::moder4_f(0b10);
	}

	/*static bool send(const uint8_t* data, uint16_t length)
	{
		return false;
	}*/

	static uint32_t setSpeed(uint32_t baudrate, bool parity)
	{
		usart3.brr = (SystemCoreClock + baudrate / 2) / baudrate;
		return SystemCoreClock / usart3.brr.load().value();
		// TODO : set parity !
	}


	static void onReceived(opsy::Callback<void(Event, uint8_t)>&& callback = opsy::Callback<void(Event, uint8_t)>())
	{
		const std::lock_guard<opsy::PriorityMutex> lock(s_mutex);

		s_onEvent = std::move(callback);

		if(s_onEvent)
		{
			usart3.cr3 <<= usart1_p::cr3_r::eie_f();
			usart3.cr1 <<= usart1_p::cr1_r::rxneie_f() || usart1_p::cr1_r::idleie_f();
		}
		else
		{
			usart3.cr3 <<= usart1_p::cr3_r::eie_f(false);
			usart3.cr1 <<= usart1_p::cr1_r::rxneie_f(false) || usart1_p::cr1_r::idleie_f(false);
		}
	}

	static void onSwitched(opsy::Callback<void(float, bool)>&& callback = opsy::Callback<void(float, bool)>(), bool both = false)
	{
		const std::lock_guard<opsy::PriorityMutex> lock(s_mutex);

		s_onSwitched = std::move(callback);

		if(s_onSwitched)
		{
			tim16.dier = tim16_p::dier_r::cc1ie_f() | tim16_p::dier_r::uie_f();
			tim16.ccer = tim16_p::ccer_r::cc1np_f(both) | tim16_p::ccer_r::cc1p_f() | tim16_p::ccer_r::cc1e_f();
		}
		else
		{
			tim16.dier = tim16_p::dier_r::ResetValue;
			tim16.ccer = tim16_p::ccer_r::ResetValue;
		}
	}

	static void uartIsrHandler()
	{
		auto isr = usart3.isr.load();

		if(isr.rxne())
			s_onEvent(Event::Data, static_cast<uint8_t>(usart3.rdr.load().rdr()));

		if(isr.idle())
		{
			s_onEvent(Event::Idle, static_cast<uint8_t>(0));
			usart3.icr = usart1_p::icr_r::idlecf_f();
		}

		if(static_cast<bool>(isr.ore()) || static_cast<bool>(isr.nf()))

		{
			s_onEvent(Event::Error, static_cast<uint8_t>(0));
			usart3.icr = usart1_p::icr_r::orecf_f() | usart1_p::icr_r::ncf_f() | usart1_p::icr_r::fecf_f();
		}

		// discard parity error
		usart3.icr = usart1_p::icr_r::pecf_f() | usart1_p::icr_r::fecf_f();
	}

	static void timIsrHandler()
	{
		auto sr = tim16.sr.load();
		tim16.sr = tim16_p::sr_r::ResetValue;

		if(sr.uif()) // overflow
			s_lastSwitch -= tim16.arr.load().arr();

		if(sr.cc1if()) // capture
		{
			auto cnt = tim16.ccr1.load().ccr1();
			assert(cnt > s_lastSwitch);
			uint32_t delta = cnt - s_lastSwitch;
			s_lastSwitch = cnt;
			auto wasHigh = !gpiob.idr.load().idr4();
			s_onSwitched(static_cast<float>(delta) * s_tickTime, wasHigh);
		}
	}

	static void set(Pull pull = Pull::None)
	{
		switch(pull)
		{
		case Pull::WeakDown:
			gpiob.moder <<= gpiob_p::moder_r::moder4_f(0b10) || atomic; // PB4 as alternate function (TIM16)
			gpioa.moder <<= gpioa_p::moder_r::moder15_f(0b00) || atomic; // PA15 as input
			gpiob.pupdr <<= gpiob_p::pupdr_r::pupdr4_f(0b10) || atomic; // weak pull down on PB4
			break;
		case Pull::WeakUp:
			gpiob.moder <<= gpiob_p::moder_r::moder4_f(0b10) || atomic; // PB4 as alternate function (TIM16)
			gpioa.moder <<= gpioa_p::moder_r::moder15_f(0b00) || atomic; // PA15 as input
			gpiob.pupdr <<= gpiob_p::pupdr_r::pupdr4_f(0b01) || atomic; // weak pull up on PB4
			break;
		case Pull::Down:
			gpiob.moder <<= gpiob_p::moder_r::moder4_f(0b10) || atomic; // PB4 as alternate function (TIM16)
			gpioa.bsrr = gpioa_p::bsrr_r::br15_f(); // drive PA15 low
			gpioa.moder <<= gpioa_p::moder_r::moder15_f(0b01) || atomic; // PA15 as output
			gpiob.pupdr <<= gpiob_p::pupdr_r::pupdr4_f(0b00) || atomic; // no pull on PB4
			break;
		case Pull::Up:
			gpiob.moder <<= gpiob_p::moder_r::moder4_f(0b10) || atomic; // PB4 as alternate function (TIM16)
			gpioa.bsrr = gpioa_p::bsrr_r::bs15_f(); // drive PA15 high
			gpioa.moder <<= gpioa_p::moder_r::moder15_f(0b01) || atomic; // PA15 as output
			gpiob.pupdr <<= gpiob_p::pupdr_r::pupdr4_f(0b00) || atomic; // no pull on PB4
			break;
		case Pull::StrongDown:
			gpiob.bsrr = gpiob_p::bsrr_r::br4_f(); // drive PB4 low
			gpiob.moder <<= gpiob_p::moder_r::moder4_f(0b01) || atomic; // PB4 as output
			gpioa.moder <<= gpioa_p::moder_r::moder15_f(0b00) || atomic; // PA15 as input
			gpiob.pupdr <<= gpiob_p::pupdr_r::pupdr4_f(0b00) || atomic; // no pull on PB4
			break;
		case Pull::StrongUp:
			gpiob.bsrr = gpiob_p::bsrr_r::bs4_f(); // drive PB4 high
			gpiob.moder <<= gpiob_p::moder_r::moder4_f(0b01) || atomic; // PB4 as output
			gpioa.moder <<= gpioa_p::moder_r::moder15_f(0b00) || atomic; // PA15 as input
			gpiob.pupdr <<= gpiob_p::pupdr_r::pupdr4_f(0b00) || atomic; // no pull on PB4
			break;
		default:
		case Pull::None:
			gpiob.moder <<= gpiob_p::moder_r::moder4_f(0b10) || atomic; // PB4 as alternate function (TIM16)
			gpioa.moder <<= gpioa_p::moder_r::moder15_f(0b00) || atomic; // PA15 as input
			gpiob.pupdr <<= gpiob_p::pupdr_r::pupdr4_f(0b00) || atomic; // no pull on PB4
			break;
		}
	}

	static void reversePolarity(bool state)
	{
		auto cr1 = usart3.cr1.load();
		usart3.cr1 = usart1_p::cr1_r::ResetValue; // disable uart before changing configuration
		usart3.cr2 = usart1_p::cr2_r::txinv_f(state) | usart1_p::cr2_r::rxinv_f(state);
		usart3.cr1 = cr1; // reset uart to previous configuration

		if(!tim16.ccer.load().cc1np()) // if not both edge
			tim16.ccer <<= tim16_p::ccer_r::cc1p_f(state); // set edge
	}

private:

	static constexpr uint32_t TargetFrequency = 10000000; // 10MHz target frequency

	static inline opsy::Callback<void(float, bool)> s_onSwitched;
	static inline opsy::Callback<void(Event, uint8_t)> s_onEvent;
	static inline opsy::PriorityMutex s_mutex{};

	static inline int32_t s_lastSwitch = 0;
	static inline float s_tickTime;
};
}
