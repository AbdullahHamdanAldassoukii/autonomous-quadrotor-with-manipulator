#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
//#include <assert.h>
#include <string.h>

#include "container.h"
#include "circular.h"

// front of circular starts at 0 and grows by adding -1 to the index
// back of circular starts at 0 and grows by adding 1 to the index

container_funcs_t circular_funcs;

struct circular_t
{
    uint32_t capacity;
    int32_t front;
    int32_t rear;
    uint32_t size;
    uint32_t elt_size;
    uint8_t  *array;
};

container_funcs_t* circular_funcs_init()
{
    circular_funcs.size_of = circular_size_of;
    circular_funcs.is_empty = circular_is_empty;
    circular_funcs.is_full = circular_is_full;
    circular_funcs.size = circular_size;
    circular_funcs.capacity = circular_capacity;
    circular_funcs.clear = circular_clear;
    circular_funcs.create = circular_create;
    circular_funcs.destroy = circular_destroy;
    circular_funcs.push_back = circular_push_back;
    circular_funcs.push_front = circular_push_front;
    circular_funcs.front = circular_front;
    circular_funcs.back = circular_back;
    circular_funcs.pop_front = circular_pop_front;
    circular_funcs.pop_back = circular_pop_back;
    circular_funcs.remove_front = circular_remove_front;
    circular_funcs.remove_back = circular_remove_back;
    circular_funcs.remove = circular_remove;
    circular_funcs.at = circular_at;
    circular_funcs.iter_create = circular_iter_create;
    circular_funcs.iter_next = circular_iter_next;
    circular_funcs.iter_destroy = circular_iter_destroy;
    return &circular_funcs;
}

uint32_t circular_size_of(void)
{
    return sizeof(circular_t);
}

uint32_t circular_size(const container_t *c)
{
    circular_t *circ = (circular_t*) c;

    return circ->size;
}

uint32_t circular_capacity(const container_t *c)
{
    circular_t *circ = (circular_t*) c;

    return circ->capacity;
}

bool circular_is_empty(const container_t *c)
{
    circular_t *circ = (circular_t*) c;

    return circ->size == 0;
}

bool circular_is_full(const container_t *c)
{
    circular_t *circ = (circular_t*) c;

    return circ->size == circ->capacity;
}

void circular_clear(container_t *c)
{
    circular_t *circ = (circular_t*) c;

    circ->size = 0;
    circ->front = 0;
    circ->rear = 0;
    memset(circ->array, 0, circ->elt_size * circ->capacity);
}

void circular_destroy(container_t *c)
{
    circular_t *circ = (circular_t*) c;

    if (circ != NULL)
    {
        free(circ->array);
        circ->array = NULL;

        free(circ);
        circ = NULL;
    }
}

container_t* circular_create(uint32_t num_max_elts, uint32_t elt_size)
{
    circular_t *circ;

    circ = (circular_t*) malloc(sizeof(circular_t));
    if (circ == NULL)
    {
        //fprintf(stderr, "Createcircular_t Error: Unable to allocate more memory.\n");
        return NULL;
    }

    circ->array = calloc( num_max_elts, elt_size );
    if (circ->array == NULL)
    {
        //fprintf(stderr, "Createcircular_t Error: Unable to allocate more memory.\n");
        free(circ);
        return NULL;
    }

    circ->capacity = num_max_elts;
    circ->elt_size = elt_size;
    circular_clear((container_t*)circ);

    return (container_t*) circ;
}

bool circular_push_back(container_t *c, const void* elt)
{
    circular_t *circ = (circular_t*) c;

    if (circular_is_full(c))
    {
        return false;
//        fprintf(stderr, "Encircular Error: The circular is full.\n");
    }
    else
    {
        memcpy(circ->array + (circ->rear * circ->elt_size), elt, circ->elt_size);

        circ->size++;
        circ->rear += 1;

        // Handle wrap around
        if( circ->rear == circ->capacity ) circ->rear = 0;
    }
    return true;
}

bool circular_push_front(container_t *c, const void* elt)
{
    circular_t *circ = (circular_t*) c;

    if (circular_is_full(c))
    {
        return false;
//        fprintf(stderr, "Encircular Error: The circular is full.\n");
    }
    else
    {
        circ->size++;
        circ->front -= 1;

        // Handle wrap around
        if( (int32_t)circ->front < 0 ) circ->front = (int32_t)circ->front + circ->capacity;

        memcpy(circ->array + (circ->front * circ->elt_size), elt, circ->elt_size);
    }
    return true;
}

// returns pointer to internal array element
const void* circular_front(const container_t *c)
{
    circular_t *circ = (circular_t*) c;

    if (!circular_is_empty(c))
    {
        void* X = (void*)(circ->array + (circ->front * circ->elt_size));
        return X;
    }
    //fprintf(stderr, "Front Error: The circular is empty.\n");

    return NULL;
}

// returns pointer to internal array element
const void* circular_back(const container_t *c)
{
    circular_t *circ = (circular_t*) c;

    if (!circular_is_empty(c))
    {
        int32_t temp = (circ->rear - 1);
        if(temp < 0) temp += circ->capacity;

        void* X = (void*)(circ->array + (temp * circ->elt_size));
        return X;
    }
    //fprintf(stderr, "Back Error: The circular is empty.\n");

    /* Return value to avoid warnings from the compiler */
    return NULL;
}

// allocates new memory and returns it
void* circular_pop_front(container_t *c)
{
    circular_t *circ = (circular_t*) c;

    void *X = NULL;

    if (circular_is_empty(c))
    {
        //fprintf(stderr, "FrontAndDecircular Error: The circular is empty.\n");
    }
    else
    {
        X = malloc( circ->elt_size );

        memcpy(X, (void*)(circ->array + (circ->front * circ->elt_size)), circ->elt_size);

        circ->size--;
        circ->front += 1;

        // Handle wrap around
        if( circ->front == circ->capacity ) circ->front = 0;

    }
    return X;
}

// allocates new memory and returns it
void* circular_pop_back(container_t *c)
{
    circular_t *circ = (circular_t*) c;

    void *X = NULL;

    if (circular_is_empty(c))
    {
        //fprintf(stderr, "PopBack Error: The circular is empty.\n");
    }
    else
    {
        X = malloc( circ->elt_size );

        circ->size--;
        circ->rear -= 1;

        // Handle wrap around
        if( (int32_t)circ->rear < 0 ) circ->rear = (int32_t)circ->rear + circ->capacity;

        memcpy(X, (void*)(circ->array + (circ->rear * circ->elt_size)), circ->elt_size);
    }
    return X;
}

void circular_remove_front(container_t *c)
{
    circular_t *circ = (circular_t*) c;

    if (circular_is_empty(c))
    {
        //fprintf(stderr, "FrontAndDecircular Error: The circular is empty.\n");
    }
    else
    {
        circ->size--;
        circ->front += 1;
        // Handle wrap around
        if( circ->front == circ->capacity ) circ->front = 0;
    }
}

void circular_remove_back(container_t *c)
{
    circular_t *circ = (circular_t*) c;

    if (circular_is_empty(c))
    {
        //fprintf(stderr, "PopBack Error: The circular is empty.\n");
    }
    else
    {
        circ->size--;
        circ->rear -= 1;
        // Handle wrap around
        if( (int32_t)circ->rear < 0 ) circ->rear = (int32_t)circ->rear + circ->capacity;
    }
}

// return true/false
bool circular_remove(container_t *c, const void *elt)
{
    // circular_remove has not yet been implemented
    while(1);
//    return false;
}

const void* circular_at(const container_t *c, uint32_t i)
{
    circular_t *circ = (circular_t*) c;

    if (i < circular_size(c))
    {
        int idx = (i + circ->front) % circ->capacity;
        void* X = (void*)(circ->array + (idx * circ->elt_size));
        return X;
    }
    //fprintf(stderr, "At Error: cant index into element %d. "
                    //"Queue has size %d\n", i, circular_size(c));

    return NULL;
}

container_iter_t* circular_iter_create(container_t *c)
{
    int32_t *i = (int32_t*) malloc(sizeof(int32_t));
    *i = ((circular_t*) c)->front;
    return (container_iter_t*) i;
}

const void* circular_iter_next(container_t *c, container_iter_t *iter)
{
    if((*((int32_t*) iter)) == ((circular_t*) c)->capacity) return NULL;
    const void* ret = ((circular_t*) c)->array + ((*((int32_t*) iter) % ((circular_t*) c)->capacity) * ((circular_t*) c)->elt_size);
    (*((int32_t*) iter))++;
    return ret;
}

void circular_iter_destroy(container_iter_t *iter)
{
    free(iter);
}

#ifdef __cplusplus
}
#endif
