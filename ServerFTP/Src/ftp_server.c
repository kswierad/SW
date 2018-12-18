#include "ftp_server.h"
//#include "memory_access.h"
//#include "client_thread.h"
#include "lwip.h"
#include "lwip/timeouts.h"

#define TASK_STACK_SIZE 1000

struct netconn *open_conn(uint16_t port) {
	struct netconn *conn;
	while((conn = netconn_new(NETCONN_TCP)) == NULL) {
		xprintf("netconn_new error, trying again\n");
		vTaskDelay(1000);
	}

    while(netconn_bind(conn, &gnetif.ip_addr, port) != ERR_OK) {
        xprintf("netconn_bind error, trying again\n");
        vTaskDelay(1000);
    }

    while(netconn_listen(conn) != ERR_OK) {
        xprintf("netconn_listen error, trying again\n");
        vTaskDelay(1000);
    }

    return conn;
}

void accept_client() {
    struct netconn *client_conn;
    xprintf("HALLO\n");
    if(netconn_accept(command_conn, &client_conn) != ERR_OK) {
        xprintf("netconn_accept error\n");
        return;
    }

    xprintf("client accepted\n");

    serve_client(client_conn);
}

void ftp_start() {
	xprintf("Server started. Waiting for address...\n");
	while(gnetif.ip_addr.addr == 0) {
        /*xprintf("IP: %u.%u.%u.%u\nPORT: %d\n",
                ip4_addr1(&gnetif.ip_addr),
                ip4_addr2(&gnetif.ip_addr),
                ip4_addr3(&gnetif.ip_addr),
                ip4_addr4(&gnetif.ip_addr),
                COMMAND_PORT);*/
		vTaskDelay(1000);
	} 

	xprintf("IP: %u.%u.%u.%u\nPORT: %d\n",
				ip4_addr1(&gnetif.ip_addr),
				ip4_addr2(&gnetif.ip_addr),
				ip4_addr3(&gnetif.ip_addr),
				ip4_addr4(&gnetif.ip_addr),
				COMMAND_PORT);

	command_conn = open_conn(COMMAND_PORT);
    data_conn = open_conn(DATA_PORT);

    while(1) {
        accept_client();
    }
}



////Paraller clients
//
//void accept_client_paraller() {
//    struct netconn *client_conn;
//
//    if(netconn_accept(command_conn, &client_conn) != ERR_OK) {
//        xprintf("netconn_accept error\n");
//        return;
//    }
//
//    if(xQueueSend(clients_queue, (void *) &client_conn, (TickType_t) 10 ) != pdPASS ) {
//        netconn_close(client_conn);
//        netconn_delete(client_conn);
//        xprintf("xQueueSend error\n");
//    }
//
//    xprintf("client accepted\n");
//
//    serve_client(client_conn);
//}
//
//void server_ftp_task(void *arg) {
//    while(1) {
//        accept_client();
//        vTaskDelay(100);
//    }
//}
//
//void ftp_start_paraller() {
//    xprintf("Server started. Waiting for address...\n");
//    while(gnetif.ip_addr.addr == 0) {
//        /*xprintf("IP: %u.%u.%u.%u\nPORT: %d\n",
//                ip4_addr1(&gnetif.ip_addr),
//                ip4_addr2(&gnetif.ip_addr),
//                ip4_addr3(&gnetif.ip_addr),
//                ip4_addr4(&gnetif.ip_addr),
//                COMMAND_PORT);*/
//        vTaskDelay(1000);
//    }
//
//    xprintf("IP: %u.%u.%u.%u\nPORT: %d\n",
//                ip4_addr1(&gnetif.ip_addr),
//                ip4_addr2(&gnetif.ip_addr),
//                ip4_addr3(&gnetif.ip_addr),
//                ip4_addr4(&gnetif.ip_addr),
//                COMMAND_PORT);
//
//    command_conn = open_conn(COMMAND_PORT);
//    data_conn = open_conn(DATA_PORT);
//
//    clients_queue = xQueueCreate(CLIENTS_QUEUE_SIZE, sizeof(struct netconn *));
//    if(clients_queue == NULL)
//        xprintf("xQueueCreate error\n");
//
//    //Create server task accepting clients
//    BaseType_t xReturned;
//    TaskHandle_t server_task;
//    xReturned = xTaskCreate(server_ftp_task, "ST1", TASK_STACK_SIZE, NULL, 16, &server_task);
//    if( xReturned != pdPASS )
//            xprintf("xTaskCreate error, server_task\n");
//
//    //Create client tasks serving clients
//    TaskHandle_t clients_tasks[CLIENTS_TASKS_NUMBER];
//    for(uint8_t i = 0; i < CLIENTS_TASKS_NUMBER; i++) {
//        char task_name[5];
//        sprintf(task_name, "CT%d", i);
//        BaseType_t xReturned = xTaskCreate(serve_client_task, task_name, TASK_STACK_SIZE, NULL, 16, &clients_tasks[i]);
//
//         if( xReturned != pdPASS )
//            xprintf("xTaskCreate error, client: %d\n", i);
//    }
//
//    vTaskStartScheduler();
//    xprintf("vTaskStartScheduler error\n");
//}