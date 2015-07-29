#ifndef WIEGAND_H_
#define WIEGAND_H_

#define CARD_DATA_LEN 6
#define WIEGAND_MAX_CARDS 146

typedef struct Card Card;
struct Card {
    uint8_t bit_len;
    uint8_t data[CARD_DATA_LEN];
};

typedef struct Wiegand_ctx Wiegand_ctx;
struct Wiegand_ctx {
    Card card_store[WIEGAND_MAX_CARDS];
    uint8_t card_count;
};

void wiegand_init(Wiegand_ctx *ctx);
void wiegand_task(void);
void add_card(uint64_t *data, uint8_t len);
void send_wiegand(uint8_t card_idx);

#endif /* WIEGAND_H_ */
