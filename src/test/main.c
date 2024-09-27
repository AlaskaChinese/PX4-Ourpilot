#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_BYTES 200

int main() {
    const char *hex_string = "55 04 ac 00 02 01 ba 66 1d 00 06 09 ff de 0a 00 df ff ff e8 03 00 fa ff ff 1a 00 00 00 00 00 c0 12 00 00 00 00 00 00 00 27 ac e2 3c 56 ed 1c 3c d2 70 3b bd 32 57 66 3e 3b cb 1b 41 93 70 61bd 25 b2 6b 41 a1 22 6c 41 da da 6b 41 6d 23 e9 23 70 dd db f7 30 3f 5f d6 31 3f ba 81 1e 3e 47 69 e2 bd 91 9b 40 40 c5 23 00 00 40 40 5c d5 1c 00 00 00 1d 00 6d 13 04 01 00 6b 0c 00 b1 9f a6 66 1d 00 16 45 01 01 ac 15 00 b4 a1 a6 66 1d 00 d3 01 01 02 48 1a 00 ca 9f a6 66 1d 00 38 ba 01 03 2b 12 00 c6 a0 a6 66 1d 00 64 40 25";
    
    unsigned char bytes[MAX_BYTES];
    int byte_count = 0;
    char *token;
    char *str = strdup(hex_string);  // �����ַ����ĸ�������Ϊstrtok���޸�ԭ�ַ���
    
    // ʹ��strtok�����ָ��ַ���
    token = strtok(str, " ");
    while (token != NULL && byte_count < MAX_BYTES) {
        // ��ʮ�������ַ���ת��Ϊ���������洢��bytes������
        bytes[byte_count++] = (unsigned char)strtol(token, NULL, 16);
        token = strtok(NULL, " ");
    }
    
    // ��ӡ���
    printf("Total bytes: %d\n", byte_count);
    for (int i = 0; i < byte_count; i++) {
        printf("bytes[%d] = 0x%02X\n", i, bytes[i]);
    }
    
    free(str);  // �ͷŶ�̬������ڴ�
    return 0;
}