#include "Joystick.hpp"
using namespace remote;


#include <cstdio>
#include <fcntl.h>
#include <unistd.h>

const double JoystickState::axis_max = 0x7fff;

Joystick::Joystick()
{
	for (int i = 0; i < JOYSTICK_AXIS_COUNT; i++)
	{
		last.axis[i] = 0;
		current.axis[i] = 0;
	}
	for (int i = 0; i < JOYSTICK_BUTTON_COUNT; i++)
	{
		last.button_state[i] = false;
		last.button_up[i] = false;
		last.button_down[i] = false;
		current.button_state[i] = false;
		current.button_up[i] = false;
		current.button_down[i] = false;
	}
}


Joystick::~Joystick()
{
	close();
}

bool Joystick::open(const char* device)
{
	fd = ::open(device, O_RDONLY);
	return fd;
}

void Joystick::close()
{
	::close(fd);
}

std::string Joystick::name()
{
	if (!fd) return "";
	
	char name[128];
	if (ioctl(fd, JSIOCGNAME (sizeof(name)), name))
    {
		name[127] = 0;
		return name;
    }
    else
	{
		return "";
	}
}

void Joystick::on_event(std::function<void(struct js_event)> action)
{
	event_action = action;
}

void Joystick::on_button(std::function<void(int, bool)> action)
{
	button_action = action;
}

void Joystick::on_axis(std::function<void(int, double)> action)
{
	axis_action = action;
}


void Joystick::loop()
{
	struct js_event e;
	while (true)
	{
		read(fd, &e, sizeof(struct js_event));
		
		switch (e.type)
		{
			case (JS_EVENT_BUTTON | JS_EVENT_INIT):
			case JS_EVENT_BUTTON:
				if (e.number < JOYSTICK_BUTTON_COUNT)
				{
					current.button_state[e.number] = e.value;
					current.button_up[e.number] = (!current.button_state[e.number] & last.button_state[e.number]);
					current.button_down[e.number] = (current.button_state[e.number] & !last.button_state[e.number]);
					
					if (e.type != (JS_EVENT_BUTTON | JS_EVENT_INIT))
						if (button_action != nullptr)
							button_action(e.number, e.value);
				}
				break;
				
			case (JS_EVENT_AXIS | JS_EVENT_INIT):
			case JS_EVENT_AXIS:
				if (e.number < JOYSTICK_AXIS_COUNT)
				{
					current.axis[e.number] = (e.value / JoystickState::axis_max);
					
					if (e.type != (JS_EVENT_AXIS | JS_EVENT_INIT))
						if (axis_action != nullptr)
							axis_action(e.number, current.axis[e.number]);
				}
				break;
				
			default:
				break;
		}
		
		if (event_action != nullptr)
			event_action(e);

		last = current;
	}
}
