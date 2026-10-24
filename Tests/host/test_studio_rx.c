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
static void cmd_putc(char c) { (void)c;echo_count++; }
static void cmd_put_prompt(void) { prompts++; }
static void cmd_history_add(const char *line) { (void)line;histories++; }
static void cmd_print_run_stamp(const char *line) { (void)line; }
static void cmd_dispatch(char *line) { (void)line;commands++; }
static void cmd_history_up(void) {}
static void cmd_history_down(void) {}
static void Cmd_StudioTargetLine(const char *line) { targets++;strcpy(last_target,line); }
#include "studio_rx_under_test.inc"
static void feed(const char *line) { while(*line)cmd_handle_char(*line++); }
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
