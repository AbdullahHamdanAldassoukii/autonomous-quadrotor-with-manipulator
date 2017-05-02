#ifndef _CIRCULAR_H_
#define _CIRCULAR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "container.h"

typedef struct circular_t circular_t;

extern container_funcs_t circular_funcs;

container_funcs_t* circular_funcs_init();
uint32_t           circular_size_of(void);
bool               circular_is_empty(const container_t *c);
bool               circular_is_full(const container_t *c);
uint32_t           circular_size(const container_t *c);
uint32_t           circular_capacity(const container_t *c);
void               circular_clear(container_t *c);
container_t *      circular_create(uint32_t num_max_elts, uint32_t elt_size);
void               circular_destroy(container_t *c);
void               circular_make_empty(container_t *c);
bool               circular_push_back(container_t *c, const void* elt);
bool               circular_push_front(container_t *c, const void* elt);
const void*        circular_front(const container_t *c);
const void*        circular_back(const container_t *c);
void*              circular_pop_front(container_t *c);
void*              circular_pop_back(container_t *c);
void               circular_remove_front(container_t *c);
void               circular_remove_back(container_t *c);
bool               circular_remove(container_t *c, const void *elt);
const void*        circular_at(const container_t *c, uint32_t i);
container_iter_t*  circular_iter_create(container_t *c);
const void*        circular_iter_next(container_t *c, container_iter_t *iter);
void               circular_iter_destroy(container_iter_t *iter);

#ifdef __cplusplus
}
#endif

#endif  /* _CIRCULAR_H_ */
