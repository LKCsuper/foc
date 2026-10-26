/* Studio 接收主机测试：静默目标路由、超长丢弃与重同步 */
/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#define CMD_BUF_SIZE 160U
static char s_cmd_buf[CMD_BUF_SIZE];
static uint8_t s_cmd_len,s_cmd_escape_state,s_cmd_history_depth,s_cmd_draft_len;
static unsigned echo_count,prompts,histories,targets,commands;
static char last_target[160];
static const struct { const char *name;const char *desc; } s_cmd_table[]={{"help","help"}};
static const unsigned s_cmd_count=1;
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 主机测试替身：向命令串口回显一个字符。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {char} c 兼容参数，本实现不使用
 * @return {void} 无
 */
static void cmd_putc(char c) { (void)c;echo_count++; }
/**
 * @description: 主机测试替身：打印带上电时基的命令提示符。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void cmd_put_prompt(void) { prompts++; }
/**
 * @description: 主机测试替身：把非空且不与上一条重复的命令写入历史环。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const char *} line 兼容参数，本实现不使用
 * @return {void} 无
 */
static void cmd_history_add(const char *line) { (void)line;histories++; }
/**
 * @description: 主机测试替身：在执行前打印命令和时间戳。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const char *} line 兼容参数，本实现不使用
 * @return {void} 无
 */
static void cmd_print_run_stamp(const char *line) { (void)line; }
/**
 * @description: 主机测试替身：按空格拆分一行命令并调用命令表。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {char *} line 兼容参数，本实现不使用
 * @return {void} 无
 */
static void cmd_dispatch(char *line) { (void)line;commands++; }
/**
 * @description: 主机测试替身：方向键上：回退到更早的一条历史命令。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void cmd_history_up(void) {}
/**
 * @description: 主机测试替身：方向键下：回到更新的历史，或恢复草稿。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void cmd_history_down(void) {}
/**
 * @description: 主机测试替身：校验并写入网页目标帧到单槽邮箱，仅在命令任务上下文调用。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const char *} line 输入文本行
 * @return {void} 无
 */
static void Cmd_StudioTargetLine(const char *line) { targets++;strcpy(last_target,line); }
#include "studio_rx_under_test.inc"
/**
 * @description: 向被测模块注入一拍输入。
 * @param {const char *} line 输入文本行
 * @return {void} 无
 */
static void feed(const char *line) { while(*line)cmd_handle_char(*line++); }
/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    unsigned i;
    feed("@C1,77,1,100,0.3,0*1234\r");
    assert(targets==1&&!strcmp(last_target,"@C1,77,1,100,0.3,0*1234"));
    assert(echo_count==0&&histories==0&&prompts==0&&s_cmd_len==0);
    feed("olstatus\r");assert(commands==1&&histories==1&&prompts==1);
    feed("@C1,77,2,100,0.3,0*1234");for(i=0;i<200;i++)cmd_handle_char('x');cmd_handle_char('\r');
    assert(targets==1&&s_cmd_len==0&&s_cmd_escape_state==0);
    feed("@C1,77,3,100,0.3,0*1234\n");assert(targets==2);
    feed("olstop\r");assert(commands==2);
    puts("studio RX: quiet target routing, oversize discard, resync and legacy command compatibility PASS");
    return 0;
}
