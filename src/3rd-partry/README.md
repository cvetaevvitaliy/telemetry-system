# 3rd-party
The source code of the project modules is stored here
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
 ┗ 📂 tinystring
   ┣ 📄 tinystring.c
   ┗ 📄 tinystring.h
```
------

### FreeRTOS source code

```
📦 FreeRTOS
 ┣ 📂 CMSIS_RTOS
 ┃ ┣ 📜 cmsis_os.c
 ┃ ┗ 📜 cmsis_os.h
 ┣ 📂 include
 ┃ ┣ 📜 FreeRTOS.h
 ┃ ┣ 📜 FreeRTOSConfig_template.h
 ┃ ┣ 📜 StackMacros.h
 ┃ ┣ 📜 croutine.h
 ┃ ┣ 📜 deprecated_definitions.h
 ┃ ┣ 📜 event_groups.h
 ┃ ┣ 📜 list.h
 ┃ ┣ 📜 mpu_prototypes.h
 ┃ ┣ 📜 mpu_wrappers.h
 ┃ ┣ 📜 portable.h
 ┃ ┣ 📜 projdefs.h
 ┃ ┣ 📜 queue.h
 ┃ ┣ 📜 semphr.h
 ┃ ┣ 📜 task.h
 ┃ ┗ 📜 timers.h
 ┣ 📂 portable
 ┃ ┣ 📂 GCC
 ┃ ┃ ┗ 📂 ARM_CM4F
 ┃ ┃ ┃ ┣ 📜 port.c
 ┃ ┃ ┃ ┗ 📜 portmacro.h
 ┃ ┗ 📂 MemMang
 ┃ ┃ ┗ 📜 heap_4.c
 ┣ 📜 croutine.c
 ┣ 📜 event_groups.c
 ┣ 📜 list.c
 ┣ 📜 queue.c
 ┣ 📜 tasks.c
 ┗ 📜 timers.c
```

------

*the document will be updated as the project develops
