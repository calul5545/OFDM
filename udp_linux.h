#ifndef UDP_LINUX_H
#define UDP_LINUX_H

// asigură compatibilitatea apelurilor funcțiilor pentru compilatoarele de c
#ifdef __cplusplus
extern "C" {
#endif

// declarare capete funcții pentru mecanismele de emisie
int udp_init_tx(void);
void udp_send(int sock, const unsigned char* data, int num_elements); 
void udp_close(int sock);

// declarare capete funcții dedicate procesului de recepție
int udp_init_rx(void); 
int udp_recv(int sock, unsigned char* buffer, int max_elements);
void udp_delay(int ms);

#ifdef __cplusplus
}
#endif

#endif