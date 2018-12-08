#ifndef GUI_PAGE_INTERFACE_H
#define GUI_PAGE_INTERFACE_H

typedef struct {
    void (*initialize)(void*);
    void (*load)(void*);
    void (*delete)(void*);
    void* arg;
} page_t;

#endif /* GUI_PAGE_INTERFACE_H */
