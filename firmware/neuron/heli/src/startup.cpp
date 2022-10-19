#include <cstdint>

#include <interrupts.hpp>
#include <InterruptVector.hpp>
#include <Scheduler.hpp>
#include "Bsp.hpp"


extern "C"
{
extern void __libc_init_array();

extern uint32_t _estack;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

uint32_t SystemCoreClock = 84000000;
}

extern "C" void Reset_Handler();

constexpr auto __attribute__((section(".isr_vector"))) vector = InterruptVector<IrqCount>(&_estack, Reset_Handler, SysTick_Handler, SVC_Handler, PendSV_Handler)
		.Register<DMA2_Stream0_IRQn, neuron::mpu_spi::isrHandler>()
		.Register<EXTI0_IRQn, neuron::mpu_exti::isrHandler>()
		.Register<USART6_IRQn, neuron::rxuart::uartIsrHandler>()
		.Register<TIM1_UP_TIM10_IRQn, neuron::rxuart::timIsrHandler>();

int main()
{
	opsy::CortexM::enableFpu();
	__libc_init_array();
	Bsp::initialize();
	opsy::Scheduler::start();
}

__attribute__((naked)) void Reset_Handler()
{
	opsy::CortexM::setMsp(&_estack);
	auto vtor = reinterpret_cast<const IsrHandler**>(reinterpret_cast<uint32_t>(&vector));
	opsy::CortexM::moveVtor(vtor);

	// initializes the data memory by copying from flash to ram
	for(auto flash = &_sidata, ram = &_sdata; ram != &_edata; flash++, ram++)
		*ram = *flash;

	// initializes the zero memory into ram
	for(auto ptr = &_sbss; ptr != &_ebss; ptr++)
		*ptr = 0;

	opsy::Hooks::boot(SystemCoreClock);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
	main();
#pragma GCC diagnostic pop

	asm volatile("bkpt 0"); // put a breakpoint because we are not supposed to arrive here
}

extern "C" void _close(){}
extern "C" void _fstat(){}
extern "C" void _getpid(){}
extern "C" void _isatty(){}
extern "C" void _kill(){}
extern "C" void _lseek(){}
extern "C" void _read(){}
