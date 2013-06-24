// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// Simple commandline menu system.
//
#include "AP_Common.h"

#include <ctype.h>
#include <string.h>
#include <stdlib.h>

#include "include/menu.h"

// statics
char Menu::_inbuf[MENU_COMMANDLINE_MAX];
Menu::arg Menu::_argv[MENU_ARGS_MAX + 1];

// constructor
Menu::Menu(const prog_char *prompt, const Menu::command *commands, uint8_t entries, FastSerial * serial, preprompt ppfunc) :
    _prompt(prompt),
    _commands(commands),
    _entries(entries),
    _ppfunc(ppfunc),
    _serial(serial)
{
}

// run the menu
void
Menu::run(void)
{
    int8_t		ret;
    uint8_t		len, i;
    uint8_t		argc;
    int			c;
    char		*s;

    // loop performing commands
    for (;;) {

        // run the pre-prompt function, if one is defined
        if ((NULL != _ppfunc) && !_ppfunc())
            return;

        // loop reading characters from the input
        len = 0;
        _serial->printf("%s] ", _prompt);
        for (;;) {
            c = _serial->read();
            if (-1 == c)
                continue;
            // carriage return -> process command
            if ('\r' == c) {
                _inbuf[len] = '\0';
                _serial->write('\r');
                _serial->write('\n');
                break;
            }
            // backspace
            if ('\b' == c) {
                if (len > 0) {
                    len--;
                    _serial->write('\b');
                    _serial->write(' ');
                    _serial->write('\b');
                    continue;
                }
            }
            // printable character
            if (isprint(c) && (len < (MENU_COMMANDLINE_MAX - 1))) {
                _inbuf[len++] = c;
                _serial->write((char)c);
                continue;
            }
        }

        // split the input line into tokens
        argc = 0;
        _argv[argc++].str = strtok_r(_inbuf, " ", &s);
        // XXX should an empty line by itself back out of the current menu?
        while (argc <= MENU_ARGS_MAX) {
            _argv[argc].str = strtok_r(NULL, " ", &s);
            if ('\0' == _argv[argc].str)
                break;
            _argv[argc].i = atol(_argv[argc].str);
            _argv[argc].f = atof(_argv[argc].str);	// calls strtod, > 700B !
            argc++;
        }

		if (_argv[0].str == NULL) {
			continue;
		}

        // populate arguments that have not been specified with "" and 0
        // this is safer than NULL in the case where commands may look
        // without testing argc
        i = argc;
        while (i <= MENU_ARGS_MAX) {
            _argv[i].str = "";
            _argv[i].i = 0;
            _argv[i].f = 0;
            i++;
        }

		bool cmd_found = false;
        // look for a command matching the first word (note that it may be empty)
        for (i = 0; i < _entries; i++) {
            if (!strcasecmp_P(_argv[0].str, (prog_char_t*)_commands[i].command)) {
                ret = _call(i, argc);
                cmd_found=true;
                if (-2 == ret)
                    return;
                break;
            }
        }

        // implicit commands
        if (i == _entries) {
            if (!strcmp(_argv[0].str, "?") || (!strcasecmp_P(_argv[0].str, (prog_char_t*)"help"))) {
                _help();
                cmd_found=true;
            } else if (!strcasecmp_P(_argv[0].str, (prog_char_t*)"exit")) {
                return;
            }
        }

        if (cmd_found==false)
        {
    		_serial->println("Invalid command, type 'help'");
    	}

    }
}

// display the list of commands in response to the 'help' command
void
Menu::_help(void)
{
    int		i;

    _serial->println("Commands:");
    for (i = 0; i < _entries; i++)
        _serial->printf("  %s\n", _commands[i].command);
}

// run the n'th command in the menu
int8_t
Menu::_call(uint8_t n, uint8_t argc)
{
    func		fn;

    //fn = (func)pgm_read_word(&_commands[n].func);
    fn = _commands[n].func;
    return(fn(argc, &_argv[0]));
}
