// #include <dfs_file.h>
// #include <dfs_posix.h>
// #include <rtdevice.h>
// #include <rtthread.h>
// #include <stdlib.h>

// #define PIN_DEBUG_SD_SAMPLE 0
// #define SD_SAMPLE_TEST_BLOCK_NUM 512

// int file_fd;

// static void fill_buffer_index(rt_uint8_t* buff, rt_uint32_t buff_length, uint8_t index_check) {
//     rt_uint32_t index;

//     for (index = 0; index < buff_length; index++) {
//         buff[index] = index + index_check;
//     }
// }

// static int openFile(uint8_t id, uint32_t flag) {
//     char file_name[64];
//     rt_snprintf(file_name, sizeof(file_name), "%s%d.bin", "log", id);
//     file_fd = open(file_name, flag, 0);

//     if (file_fd < 0) {
//         rt_kprintf("Failed to open file: %s\r\n", file_name);

//         return -1;
//     }
//     return 0;
// }

// static int writeFile(uint8_t* buf, uint16_t size) { return write(file_fd, buf, size); }

// static int closeFile(void) {
//     if (file_fd) {
//         close(file_fd);
//     }
// }

// static int testWriteFile(uint16_t block_num,uint8_t sec_time) {
//     uint8_t file_id = 1;
//     if (openFile(file_id, O_RDWR | O_CREAT | O_TRUNC) != 0) {
//         return -1;
//     }
//     uint16_t block = 512;
//     // rt_uint8_t *write_buff, *read_buff;
//     uint16_t times_wirte_success = 0;
//     uint32_t buf_size = block_num * block;

//     // write_buff = rt_malloc(buf_size);
//     static uint8_t write_buff[512*200];
//     // if (write_buff == RT_NULL) {
//     //     rt_kprintf("no memory for write buffer!\n");
//     //     rt_free(write_buff);
//     //     return -1;
//     // }

//     static uint8_t read_buff[512*200];
//     // read_buff = rt_malloc(buf_size);
//     // if (read_buff == RT_NULL) {
//     //     rt_kprintf("no memory for read_buff buffer!\n");
//     //     rt_free(read_buff);
//     //     return -1;
//     // }

//     uint32_t time_now = rt_tick_get();
//     uint32_t time_duing = 0;
//     while (1) {
//         fill_buffer_index(write_buff, buf_size, times_wirte_success);
//         uint16_t write_ret = writeFile(write_buff, buf_size);
//         fsync(file_fd);
//         if (write_ret != buf_size) {
//             rt_kprintf("write file fail,now time is %d during time %d\r\n", times_wirte_success, time_duing);
//             // break;
//         } else {
//             times_wirte_success++;
//         }
//         uint32_t time_after = rt_tick_get();
//         time_duing = time_after - time_now;
//         if (time_duing > (1000 * sec_time)) {
//             break;
//         }
//     }

//     uint32_t total_byte = times_wirte_success * buf_size;
//     uint16_t speed = (total_byte / 1024) / sec_time;
//     uint32_t bps = speed * 1024 * 10;
//     rt_kprintf("write file over!, during time %d ms\r\n", time_duing);
//     rt_kprintf("per write block is %d \r\n", buf_size);
//     rt_kprintf("total %d times_wirte_success %d byte,in %d s\n", times_wirte_success, total_byte, sec_time);
//     rt_kprintf("speed %dKB/s, baudrate %d\r\n", speed, bps);

//     closeFile();
//     if (openFile(file_id, O_RDONLY) != 0) {
//         rt_kprintf("open file fail,read check fail\r\n");
//         rt_free(write_buff);
//         rt_free(read_buff);
//         return -1;
//     }

//     uint16_t time_read_check = 0;
//     for (uint16_t i = 0; i < times_wirte_success; i++) {
//         memset(read_buff, 0, buf_size);
//         uint32_t read_size = read(file_fd, read_buff, buf_size);
//         if (read_size != buf_size) {
//             rt_kprintf("read fail in %d times_wirte_success,ret is %d\r\n", i, read_size);
//             break;
//         }
//         fill_buffer_index(write_buff, buf_size, i);
//         if (rt_memcmp(write_buff, read_buff, buf_size) == 0) {
//             // rt_kprintf("read check ok,in %d times_wirte_success\r\n", i + 1);
//             time_read_check++;
//         } else {
//             rt_kprintf("read check fail,in %d times_wirte_success\r\n", i + 1);
//             break;
//         }
//     }
//     if (time_read_check == times_wirte_success) {
//         rt_kprintf("all read check ok\r\n");
//     }

//     // rt_free(write_buff);
//     // rt_free(read_buff);

//     closeFile();
//     return 0;
// }

// static int file_speed_test(int argc, char* argv[]) {
//     if (argc != 3) {
//         rt_kprintf("please enter sd0 test_count\n");
//         return -1;
//     }
//     uint16_t block_num = atoi(argv[1]);
//     uint8_t sec_time = atoi(argv[2]);

// #if PIN_DEBUG_SD_SAMPLE
//     test_pinToggle1();
// #endif
//     testWriteFile(block_num,sec_time);
// #if PIN_DEBUG_SD_SAMPLE
//     test_pinToggle1();
// #endif

//     return RT_EOK;
// }
// /* 导出到 msh 命令列表中 */
// MSH_CMD_EXPORT(file_speed_test, file_speed_test sample);
