#ifndef _CONTAINER_H_
#define _CONTAINER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct container_t container_t;
typedef struct container_iter_t container_iter_t;

typedef struct container_funcs_t container_funcs_t;
struct container_funcs_t
{
    uint32_t          (*size_of)(void);
    bool              (*is_empty)(const container_t *c);
    bool              (*is_full)(const container_t *c);
    uint32_t          (*size)(const container_t *c);
    uint32_t          (*capacity)(const container_t *c);
    void              (*clear)(container_t *c);
    container_t *     (*create)(uint32_t num_max_elts, uint32_t elt_size);
    void              (*destroy)(container_t *c);
    bool              (*push_back)(container_t *c, const void* elt);
    bool              (*push_front)(container_t *c, const void* elt);
    const void*       (*front)(const container_t *c);
    const void*       (*back)(const container_t *c);
    void*             (*pop_front)(container_t *c);
    void*             (*pop_back)(container_t *c);
    void              (*remove_front)(container_t *c);
    void              (*remove_back)(container_t *c);
    bool              (*remove)(container_t *c, const void *elt);
    const void*       (*at)(const container_t *c, uint32_t i);
    container_iter_t* (*iter_create)(container_t *c);
    const void*       (*iter_next)(container_t *c, container_iter_t *iter);
    void              (*iter_destroy)(container_iter_t *iter);
};

#ifdef __cplusplus
}
#endif

#endif // _CONTAINER_H_
