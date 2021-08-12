#pragma once

#include <array>
#include <cstdint>

#include <Enums.hpp>
#include <Channels.hpp>

template<typename RxUart, typename Manager>
class SRXL
{
public:

	SRXL(Manager& manager) : m_manager(manager)
	{

	}

	void Start()
	{
		RxUart::reversePolarity(false);
		RxUart::set(Pull::Down);
		RxUart::setSpeed(Baudrate);
		RxUart::onReceived([&](Event event, uint8_t value) { eventReceived(event, value); });
	}


private:

	void eventReceived(Event event, uint8_t value)
	{
		switch(event)
		{
		case Event::Data:
		{
			if(m_index == 0 && value != Header12 && value != Header16 && value != HeaderSpek) // first byte should be header
				m_skip = true;

			if(!m_skip)
			{
				m_buffer <<= 8;
				m_buffer |= value;

				switch(m_index)
				{
					case 2:
						m_manager.set(Channel::Throttle, get(0));
						break;
					case 4:
						m_manager.set(Channel::Aileron, get(0));
						break;
					case 6:
						m_manager.set(Channel::Elevator, get(0));
						break;
					case 8:
						m_manager.set(Channel::Rudder, get(0));
						break;
					case 10:
						m_manager.set(Channel::Gain, get(0) * GainFactor);
						break;
					case 12:
						m_manager.set(Channel::Pitch_Aux0, get(0));
						break;
					case 14:
						m_manager.set(Channel::Setup, get(0));
						break;
					case 16:
						m_manager.set(Channel::Aux1, get(0));
						break;
					case 18:
						m_manager.set(Channel::Aux2, get(0));
						break;
					case 20:
						m_manager.set(Channel::Aux3, get(0));
						break;
					case 22:
						m_manager.set(Channel::Aux4, get(0));
						break;
					case 24:
						m_manager.set(Channel::Aux5, get(0));
						break;
					default:
						break;
				}
			}

			m_index++;
			break;
		}
		case Event::Error:
			m_skip = true;
			break;
		case Event::Idle:
			m_skip = false;
			m_index = 0;
			break;
		default:
			break;
		}
	}

	float get(uint8_t shift)
	{
		auto ticks = static_cast<int32_t>((m_buffer >> shift) & Mask);
		return static_cast<float>(ticks - Center) * Ratio;
	}

	static constexpr uint32_t Baudrate = 115200;
	static constexpr uint8_t Header12 = 0xA1;
	static constexpr uint8_t Header16 = 0xA2;
	static constexpr uint8_t HeaderSpek = 0xA5;
	static constexpr uint32_t Mask = 0xFFF;
	static constexpr int32_t Center = 2048;  // The channel value that represents 1500us output
	static constexpr float Ratio = 1.f / 1225.f;  // The ratio required for scaling the output of -1 and +1 with channel values that represent 1081.25us and 1918.75us respectively
	static constexpr float GainFactor = 1.f / 1.19f;

	Manager& m_manager;
	uint32_t m_buffer = 0;
	uint32_t m_index = 0;
	bool m_skip = true;
};
