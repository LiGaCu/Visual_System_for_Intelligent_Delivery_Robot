#include <stdio.h>
 
#ifdef _WIN32 //Linux platform
    #include <conio.h>
#else
    #include <termios.h>
#endif
 
char get1char(void)
{
 
#ifdef _WIN32
        // Do nothing
#else   // 保存并修改终端参数
    struct termios stored_settings;
    struct termios new_settings;
    tcgetattr (0, &stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    new_settings.c_cc[VMIN] = 1;
    tcsetattr (0, TCSANOW, &new_settings);
#endif
 
    int ret = 0;
    char c;
 
#ifdef _WIN32
    c = getch();
#else
    c = getchar();
    putchar('\b'); // 删除回显
#endif
 
    printf("input:  [%c]\n", c);
 
#ifdef _WIN32
    // Do nothing
#else
    tcsetattr (0, TCSANOW, &stored_settings); // 恢复终端参数
#endif
 
    return c; 
}