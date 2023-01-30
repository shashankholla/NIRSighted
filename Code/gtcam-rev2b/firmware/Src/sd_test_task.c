/**
 * This file contains a test thread that writes data at a rate of 8192B / 40ms to the SD card.
 */

#include <string.h>

#include "sd_test_task.h"
#include "sd_driver_task.h"

#include "cmsis_os.h"

const char* lorem_ipsum_text = __DATE__  " " __TIME__ " Lorem ipsum dolor sit amet, consectetur adipiscing elit. Donec viverra mauris et lorem blandit, ac scelerisque dui commodo. Vivamus ut diam vel mauris laoreet porta at pulvinar massa. Cras quis elementum lorem. Cras consequat, leo id lacinia commodo, purus lacus aliquam justo, eu mattis velit tellus id turpis. Donec sodales commodo metus ac suscipit. Integer cursus nunc a ligula fermentum porta. Nam convallis sit amet sapien at eleifend. Suspendisse sit amet libero ante.\n"

"Mauris non sem et est pulvinar maximus quis at sem. Cras sed mattis urna. In bibendum vehicula sem, sit amet aliquet odio fringilla et. Pellentesque ipsum metus, vehicula in turpis ut, sagittis pretium dolor. Donec laoreet accumsan tincidunt. Integer eu porta ligula. Suspendisse ullamcorper nulla vitae metus congue, eget feugiat ipsum tempus. Curabitur porttitor luctus lobortis. Duis egestas iaculis urna, ac iaculis urna rhoncus eu. Suspendisse cursus elit nec efficitur imperdiet. In gravida vehicula tristique. Cras ipsum mi, egestas luctus gravida et, vestibulum sed purus. Etiam tempor laoreet augue vel venenatis.\n"

"Donec rutrum ornare lorem. Duis ut enim sodales, hendrerit orci vitae, posuere lectus. Morbi odio nibh, auctor eget porttitor in, porta id metus. In vel consequat nulla, in tempor ante. Proin suscipit ex massa, ac dignissim sem placerat in. Mauris non nisi massa. Donec lacinia rutrum quam volutpat volutpat. Praesent dapibus ac tortor vel viverra. Donec leo tellus, fringilla sit amet velit nec, tincidunt lobortis felis. Phasellus tincidunt mollis nulla. Donec interdum, ex sit amet aliquet auctor, risus velit efficitur nibh, et feugiat nulla tellus ut augue.\n"

"Morbi ut malesuada nibh. Quisque gravida convallis risus, ut vulputate augue aliquam eget. Donec sodales interdum pharetra. Praesent fermentum nec diam sed vulputate. Etiam nibh ligula, sodales eu nisl at, interdum porttitor leo. Etiam orci augue, blandit id mauris id, rhoncus pretium nisl. Proin convallis, erat in mattis tristique, magna libero dictum tortor, ut mollis justo justo sed orci. Fusce mauris augue, efficitur at varius eu, eleifend id urna. Morbi dignissim tortor sed laoreet semper. Fusce maximus venenatis odio et hendrerit. Aenean cursus, urna eget commodo fringilla, turpis lorem semper odio, quis porta ligula lacus eget dui.\n"

"Maecenas semper felis quis augue volutpat sagittis. Etiam eu elit ac metus hendrerit consequat at sit amet dui. Fusce nec nunc consequat, accumsan metus sit amet, molestie orci. Integer ac arcu risus. Donec aliquam eros tortor, sit amet luctus purus congue sit amet. Donec eu facilisis tellus. Duis aliquam convallis justo eu dignissim. Nam mollis blandit augue aliquet pretium. Praesent dictum convallis leo, et eleifend dolor gravida sed. Vivamus tristique convallis tellus, sed finibus sapien lacinia vitae. Class aptent taciti sociosqu ad litora torquent per conubia nostra, per inceptos himenaeos. Suspendisse porttitor urna ut erat aliquam, at rutrum turpis efficitur. Praesent maximus nisl sed lorem tristique blandit.\n"

"Praesent ut turpis sed ligula pulvinar ultricies. Etiam ornare vel quam dignissim fringilla. Phasellus tempus suscipit vehicula. Integer mi nibh, convallis sed orci aliquet, tempor pellentesque mi. Vivamus tincidunt leo nec ullamcorper aliquet. Etiam eu eros mollis, bibendum nulla eu, commodo urna. Nulla vitae eros ac erat maximus efficitur. Donec feugiat dignissim elit, eget pulvinar nunc mollis ut.\n"

"Mauris a orci quis ipsum mollis tincidunt. Nam libero nibh, pellentesque at malesuada eget, tempor ut turpis. Cras et libero lectus. Vestibulum euismod bibendum nisi, eu luctus mi tempor in. Praesent tincidunt viverra urna, imperdiet molestie enim vulputate sit amet. Suspendisse id feugiat neque. Aliquam erat volutpat. Donec condimentum lacus pellentesque accumsan feugiat. Nulla facilisi. Suspendisse vitae turpis quis enim condimentum malesuada a id lorem. Sed tincidunt finibus erat, fermentum fringilla massa ultrices vitae. Cras dui purus, iaculis sed venenatis quis, eleifend nec dolor. In vel erat eu mi interdum ultrices ut vitae lectus. Ut gravida mattis nunc et ultricies. Mauris in magna ultricies, posuere nunc eget, tempor risus. Nam in ante ullamcorper odio porta rutrum.\n"

"Aliquam a neque vel dolor fermentum hendrerit sed eu dolor. Donec vel massa velit. In eget pharetra nulla. Aliquam erat volutpat. Nulla ut sollicitudin magna. Mauris elementum eleifend lacus, ut volutpat ante laoreet nec. Duis nec posuere libero, nec malesuada nibh. Vivamus sed tellus massa. Fusce ullamcorper eu mi id hendrerit. Proin id rhoncus sem, at condimentum eros. Nam at euismod justo.\n"

"In varius arcu non augue congue finibus. Integer dui turpis, vulputate sit amet augue et, porttitor tincidunt turpis. Pellentesque habitant morbi tristique senectus et netus et malesuada fames ac turpis egestas. Curabitur eu risus vitae arcu venenatis elementum in id metus. Aliquam vehicula dolor tortor, at placerat libero congue nec. Suspendisse suscipit bibendum ligula sagittis accumsan. Aenean nisi turpis, elementum eget elit sit amet, varius egestas lorem. Fusce eget cursus ante. Sed facilisis gravida mattis. Duis non viverra metus. Suspendisse at mauris varius, luctus neque et, rutrum dolor. Phasellus venenatis blandit lectus.\n"

"Morbi accumsan, orci sed ullamcorper dapibus, mauris sapien vehicula neque, et tincidunt odio tellus sed leo. Donec condimentum eros non purus tempor placerat. Maecenas varius lorem sem, eu commodo velit pellentesque a. Cras ligula est, tristique vel ligula sodales, vehicula fermentum ipsum. Quisque urna magna, tempus volutpat risus sed, rutrum tristique velit. Nunc ut malesuada arcu. Phasellus vitae gravida erat. Nullam suscipit ligula vitae pretium consectetur. Nam dictum, tortor vel vehicula convallis, quam felis dapibus felis, sit amet aliquet diam enim non libero. Suspendisse pretium consectetur quam vel aliquet. Proin facilisis vehicula libero. Nulla sed libero tincidunt, dictum libero quis, pretium metus.\n"

"Cras velit est, viverra et diam eget, tempus pulvinar odio. Nunc imperdiet suscipit orci, mattis consequat felis. Nulla id augue ut est faucibus maximus a ac diam. Aenean urna tellus, vulputate et cursus eu, rutrum et libero. Suspendisse semper urna nec neque fermentum, quis dictum lorem tempus. Nulla porttitor scelerisque nisi, ac tristique erat pharetra vel. Phasellus id elit justo. Vivamus egestas augue id lectus fringilla, at ornare velit tempus. Donec pharetra pellentesque lorem. Morbi aliquam ut sem in tempor. Pellentesque quis rhoncus justo, vitae egestas ante. Ut ac libero at purus consequat viverra a at sapien. Pellentesque nec arcu pretium, porta tortor in, dictum urna. Vivamus at mi urna. Phasellus lobortis dolor feugiat purus scelerisque interdum. Interdum et malesuada fames ac ante ipsum primis in faucibus.\n"

"Nulla tortor turpis, aliquet ut porttitor ut, viverra ut dui. Quisque vel neque a purus pharetra fringilla quis quis neque. Donec et congue augue, vitae pharetra dui. Duis efficitur tortor sed porttitor consequat. Phasellus et nisl dolor. Proin eget nisl a nisi consectetur lacinia sit amet sed nulla. In hac habitasse platea dictumst.\n"

"Donec imperdiet convallis mi ut porttitor. Curabitur libero turpis, pharetra a pretium eu, dignissim quis mauris. Proin pulvinar risus quis lacinia finibus. In id facilisis magna. Sed at tortor tellus. Mauris eget magna sit amet velit interdum suscipit non non dui. Maecenas cursus, lorem eu malesuada ornare, urna erat volutpat tortor, non viverra urna tellus nec ligula. Phasellus blandit et dolor gravida pharetra. Aliquam erat volutpat. Pellentesque bibendum pulvinar tellus, ac pretium ante scelerisque at.\n"

"Praesent eu nunc quam. Ut eu vestibulum turpis. Cras consectetur lectus at ante suscipit, quis feugiat purus ultricies. Nam quis dictum velit. Vestibulum eu enim porttitor, scelerisque arcu quis, viverra purus. Aliquam semper nisl ac viverra eleifend. Vestibulum eu risus pretium, semper dolor id, pharetra nibh. Integer metus turpis, placerat vel volutpat sit amet, rhoncus et erat. Maecenas pharetra quam nunc, vitae commodo dolor efficitur eu. Vestibulum vehicula metus finibus est feugiat ultrices. Donec ut tristique nibh, ac eleifend.";

extern QueueHandle_t sd_request_queue;

static void my_itoa(char* s, uint32_t n)
{
    int i = 0;
    do {
        s[i] = n % 10 + '0';
        n /= 10;
    } while (n);

    for (int j = 0; j < (i >> 1); j++) {
        char tmp = s[j];
        s[j] = s[i - j];
        s[i - j] = tmp;
    }
}

static void my_itoa_padded(char* s, uint32_t n, int len)
{
    for (int i = (len - 1); i >= 0; i--) {
        s[i] = n % 10 + '0';
        n /= 10;
    }
}

static uint8_t zeros[8192] = { 0 };

void sd_test_task(void* arg)
{
    #if 0
    while(1) taskYIELD();

    char* str = arg;
    while(1) {
        uint8_t idx = 0;
        while(1) {
            volatile USART_TypeDef* uart = UART4;
            if (READ_BIT(UART4->ISR, USART_ISR_TXE_TXFNF)) {
                UART4->TDR = str[idx];
                idx++;
                if (str[idx] == '\0') {
                    idx = 0;
                    taskYIELD();
                }
            }
        }
    }
#endif

#if 1
    while(0) {
        taskYIELD();
    }

    // sloppy hack: wait for about half a second for the sd driver task to start up; make sure that
    // the request queue isn't null when we start.
    vTaskDelay(200);

    sd_test_task_state_t* s = (sd_test_task_state_t*)arg;

    uint32_t addr = s->target_addr;

    sd_driver_rw_request_t req;

    uint32_t cnt = 0;
    while(1) {
        // Write some status info to the sd card about the current time and current queue depth.
        // This will let us validate write speeds.
        uint32_t tick_count = xTaskGetTickCount();
        uint32_t queue_depth = uxQueueMessagesWaiting(sd_request_queue);

        uint8_t strbuf[512];
        memset(strbuf, ' ', sizeof(strbuf));
        strcpy(strbuf, pcTaskGetName(xTaskGetCurrentTaskHandle()));
        strcpy((char*)strbuf + 32, "count: ");
        my_itoa_padded((char*)strbuf + 40, cnt++, 5);


        strcpy((char*)strbuf + 64, "Current time in ticks: ");
        my_itoa_padded((char*)strbuf + 64 + 25, tick_count, 5);

        strcpy(strbuf + 96, "Number of items in queue: ");
        my_itoa_padded((char*)strbuf + 27 + 96, queue_depth, 5);

        strcpy(strbuf + 128, __DATE__ " " __TIME__);

        req.direction = SD_DRIVER_DIRECTION_WRITE;
        req.nblocks = 1;
        req.card_addr = addr;
        req.buf = strbuf;
        sd_driver_enqueue_request(&req);

        addr += req.nblocks;

        // write our lorem ipsum string (8192 B)
        req.direction = SD_DRIVER_DIRECTION_WRITE;
        req.nblocks = 16;
        req.card_addr = addr;
        //req.buf = lorem_ipsum_text;
        req.buf = zeros;
        sd_driver_enqueue_request(&req);

        addr += req.nblocks;

        // wait for 4 / 400 of a second.
        vTaskDelay(10);
    }
#endif
}
