#pragma once

#include <cstdint>
#include <type_traits>
#include <array>
#include <cassert>

#include <opsy.hpp>
#include <Hooks.hpp>

using IsrHandler = void();

template<std::size_t PeripheralIrqs>
class InterruptVector
{
public:

	constexpr InterruptVector(
		uint32_t* endOfStack,
		IsrHandler* reset,
		IsrHandler* systick = nullptr,
		IsrHandler* serviceCall = nullptr,
		IsrHandler* pendingService = nullptr,
		IsrHandler* nonMaskableInterrupt = nullptr,
		IsrHandler* hardFault = nullptr,
		IsrHandler* memoryManagement = nullptr,
		IsrHandler* busFault = nullptr,
		IsrHandler* usageFault = nullptr,
		IsrHandler* debugMonitor = nullptr) :
            m_system(
            		endOfStack, reset, systick, serviceCall,
					pendingService, nonMaskableInterrupt, hardFault, memoryManagement,
					busFault, usageFault, debugMonitor)
	{
	}

    template<IRQn_Type IRQ, IsrHandler* handler>
    constexpr auto Register() const -> InterruptVector
    {
        static_assert(IRQ < PeripheralIrqs);
		auto tmp = m_peripherals;
		tmp[IRQ] = opsy::Hooks::decorateIsr<handler>();
		return InterruptVector(m_system, tmp);
    }

private:

    struct SystemIrq
    {
    	constexpr SystemIrq(
    			uint32_t* endOfStack,
    			IsrHandler* reset,
    			IsrHandler* systick,
    			IsrHandler* serviceCall,
    			IsrHandler* pendingService,
    			IsrHandler* nonMaskableInterrupt,
    			IsrHandler* hardFault,
    			IsrHandler* memoryManagement,
    			IsrHandler* busFault,
    			IsrHandler* usageFault,
    			IsrHandler* debugMonitor) :
    					EndOfStack(endOfStack),
						Reset(reset),
						NonMaskableInterrupt(nonMaskableInterrupt),
						HardFault(hardFault),
						MemoryManagement(memoryManagement),
						BusFault(busFault),
						UsageFault(usageFault),
						ServiceCall(serviceCall),
						DebugMonitor(debugMonitor),
						PendingService(pendingService),
						Systick(systick) {}

        uint32_t*   EndOfStack;
        IsrHandler* Reset;
        IsrHandler* NonMaskableInterrupt;
        IsrHandler* HardFault;
        IsrHandler* MemoryManagement;
        IsrHandler* BusFault;
        IsrHandler* UsageFault;
        const std::array<IsrHandler*, 4> _0 = {nullptr,nullptr,nullptr,nullptr};
        IsrHandler* ServiceCall;
        IsrHandler* DebugMonitor;
        const std::array<IsrHandler*, 1> _1 = {nullptr};
        IsrHandler* PendingService;
        IsrHandler* Systick;
    };

    constexpr InterruptVector(const SystemIrq system, const std::array<IsrHandler*, PeripheralIrqs> peripherals) :
            m_system(system), m_peripherals(peripherals)
        {

        }

public:
    const SystemIrq m_system;
    std::array<IsrHandler*, PeripheralIrqs> m_peripherals{{}};
};
