#ifndef WIEGAND_H_
#define WIEGAND_H_

#define WIEGAND_MAX_CARD_LEN 44
#define WIEGAND_MAX_CARDS 3

    
struct card {
    uint8_t bit_len;
    uint8_t data[(WIEGAND_MAX_CARD_LEN/8)+1];
};

struct wiegand_ctx {
    struct card card_store[WIEGAND_MAX_CARDS];
    uint8_t card_count;
};

void wiegand_init(struct wiegand_ctx *ctx);
void wiegand_task(void);
void add_card(uint64_t *data, uint8_t len);
#endif /* WIEGAND_H_ */
