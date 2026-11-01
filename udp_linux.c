#include "udp_linux.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

// funcție utilizată pentru pauze de procesare in milisecunde
void udp_delay(int ms) { usleep(ms * 1000); }

// inițializarea socket-ului pe emisie
int udp_init_tx(void) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return -1;
    
    // configurare setări adresă sursă
    struct sockaddr_in srcaddr;
    memset(&srcaddr, 0, sizeof(srcaddr));
    srcaddr.sin_family = AF_INET;
    srcaddr.sin_addr.s_addr = inet_addr("10.0.0.1"); // plecare t8
    srcaddr.sin_port = 0;
    bind(sock, (struct sockaddr *)&srcaddr, sizeof(srcaddr));
    
    return sock;
}

// trimiterea efectivă date convertite octet cu octet
void udp_send(int sock, const unsigned char* data, int num_elements) {
    if (sock < 0) return;
    
    // declarare țintă destinație pachete udp
    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons(5005);
    dest.sin_addr.s_addr = inet_addr("10.0.0.2"); // țintă r4
    
    // num_elements reprezintă direct numarul de bytes (uint8)
    sendto(sock, data, num_elements, 0, (struct sockaddr*)&dest, sizeof(dest));
}

// inițializare socket dedicat ascultării canalului de rețea
int udp_init_rx(void) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return -1;

    // permisiune refolosire port de ascultare
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // alocare adresă ascultare
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr("10.0.0.2"); 
    servaddr.sin_port = htons(5005);

    bind(sock, (const struct sockaddr *)&servaddr, sizeof(servaddr));
    return sock;
}

// preluare pachete stocate temporar de sistemul de operare
int udp_recv(int sock, unsigned char* buffer, int max_elements) {
    if (sock < 0) return -1;
    // recvfrom așteaptă pachetele și le pune în buffer ca bytes
    int n_bytes = recvfrom(sock, buffer, max_elements, 0, NULL, NULL);
    if (n_bytes <= 0) return 0; 
    return n_bytes;
}

// terminare sesiune pentru eliberarea portului linux
void udp_close(int sock) { if (sock >= 0) close(sock); }