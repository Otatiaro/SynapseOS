#pragma once

#include <chrono>

#include <rcc.hpp>
#include <pwr.hpp>
#include <flash.hpp>
#include <gpio.hpp>
#include <scb.hpp>

#include <CortexM.hpp>

#include <drivers/mpuexti.hpp>
#include <drivers/mpuspi.hpp>
#include <drivers/leds.hpp>
#include <drivers/governor.hpp>
#include <drivers/clock.hpp>
#include <drivers/rxuart.hpp>

#include <mpu9250.hpp>

#include <Channels.hpp>

#include <receivers/Scanner.hpp>
#include <receivers/Sbus.hpp>
#include <Enums.hpp>

using namespace STM32F401;
using namespace std::chrono_literals;

using sensor = mpu9250<neuron::mpu_spi>;

constexpr opsy::IsrPriority MpuPriority = opsy::IsrPriority(0xFF);

class Bsp
{
public:

	static void initialize()
	{
		neuron::clock::startClock(8, 336, 4, 7);

		neuron::rxuart::init(opsy::IsrPriority(0xFF));
		neuron::leds::init();
		neuron::governor::init();

		neuron::leds::set(neuron::leds::mode::separate);
		neuron::leds::set(neuron::leds::green::blinkSlow);

		sbus.Start();

		m_task.start([]()
		{
			//auto mpuSelf = sensor::selfTest<neuron::mpu_exti>(MpuPriority);


			while(true)
			{
				opsy::sleep_for(10ms);
				m_channels.update();
				auto throttle = m_channels.get(Channel::Throttle);
				neuron::leds::set(throttle > 0.f ? 2 : 1);
			}

			/*auto found = sensor::init(MpuPriority);
			assert(found);
			sensor::setGyro(GyroRange::r1000, GyroFilterOdr::lp3600_8k, 0);
			//sensor::setIntPin(false, false, StatusClear::any_read, true);

			Vector<3> gyro;
			while(true)
			{
				opsy::sleep_for(10ms);
				gyro = sensor::getGyro(GyroRange::r1000);
			}*/
		}, "Test task");
	}


private:

	static inline opsy::Task<512> m_task;

	static inline Channels m_channels = Channels(200ms, 100, 20, 10);

	static inline opsy::ConditionVariable rxFound{opsy::IsrPriority(0xFF)};
	static inline Scanner<neuron::rxuart> scanner { 10, rxFound };
	static inline Sbus<neuron::rxuart, Channels> sbus{ m_channels };

};
