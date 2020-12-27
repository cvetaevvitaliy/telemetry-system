# 3rd-party
The source code of the project modules is stored here
------

### CLI - Command Line Interface
Compact & Simple Command Line Interface for microcontrollers <br>
Uses ~2Kb Flash memory, ~1Kb RAM memory

```
📂 ./cli
 ┣ 📂l ib
 ┃ ┣ 📄 cli_input.c
 ┃ ┣ 📄 cli_input.h
 ┃ ┣ 📄 cli_log.c
 ┃ ┣ 📄 cli_log.h
 ┃ ┣ 📄 cli_queue.c
 ┃ ┣ 📄 cli_queue.h
 ┃ ┣ 📄 cli_time.c
 ┃ ┗ 📄 cli_time.h
 ┣ 📄 cli.c
 ┣ 📄 cli.h
 ┣ 📄 cli_config.h       - config CLI header file
 ┣ 📄 cli_io.c
 ┗ 📄 cli_io.h           - add input/outpur charaters fom/in IO stream
```

For add new command to CLI, need call function `cli_add_new_cmd` <br>

Where 
```
 * @param name      - input name commands
 * @param fcn       - callback execute function
 * @param argc      - min count arguments
 * @param mode      - execute mode, see CLI_Type_Mode_Cmd_t struc 
 * @param descr     - description
 * @return result   - append command

cli_add_new_cmd (const char* name, uint8_t (*fcn)(), uint8_t argc, CLI_Type_Mode_Cmd_t mode, const char* descr);
```
------

### Helper library
`tinyprintf` -  light weight implementation sprintf for embedded <br>
`tinystring` -  light weight implementation string for embedded 

```
📂 helper-library
 ┣ 📂 tinyprintf
 ┃ ┣ 📄 tinyprintf.c
 ┃ ┗ 📄 tinyprintf.h
 ┃
 ┗ 📂t inystring
   ┣ 📄 tinystring.c
   ┗ 📄 tinystring.h
```
------

*the document will be updated as the project develops
