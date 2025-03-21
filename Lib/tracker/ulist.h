 /**
 ******************************************************************************
 * @file    ulist.h
 * @author  GPM Application Team
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef ULIST_H
#define ULIST_H

#include <stdlib.h>
#include <stddef.h>

struct ulist {
  struct ulist *next;
  struct ulist *prev;
};

#define container_of(ptr, type, member) (type *) ((unsigned char *)ptr - offsetof(type,member))

#define ulist_init_head(ptr) do { \
        (ptr)->next = (ptr); \
        (ptr)->prev = (ptr); \
} while (0)

static inline void ulist_add_internal(struct ulist *item, struct ulist *prev, struct ulist *next)
{
  next->prev = item;
  item->next = next;
  item->prev = prev;
  prev->next = item;
}

static inline void ulist_add(struct ulist *item, struct ulist *head)
{
  ulist_add_internal(item, head, head->next);
}

static inline void ulist_add_tail(struct ulist *item, struct ulist *head)
{
  ulist_add_internal(item, head->prev, head);
}

static inline void ulist_del(struct ulist *item)
{
  item->prev->next = item->next;
  item->next->prev = item->prev;
  item->next = NULL;
  item->prev = NULL;
}

static inline void ulist_move(struct ulist *item, struct ulist *head)
{
  ulist_del(item);
  ulist_add(item, head);
}

static inline void ulist_move_tail(struct ulist *item, struct ulist *head)
{
  ulist_del(item);
  ulist_add_tail(item, head);
}

static inline int ulist_is_empty(const struct ulist *head)
{
  return head->next == head;
}

#define ulist_for_each(pos, head) for (pos = (head)->next; pos != (head); pos = pos->next)

#define ulist_for_each_safe(pos, n, head) for (pos = (head)->next, n = pos->next; pos != (head); pos = n, n = pos->next)

#define ulist_entry(ptr, type, member) container_of(ptr, type, member)

#define ulist_for_each_entry(pos, head, member)                               \
        for (pos = ulist_entry((head)->next, typeof(*pos), member);           \
             &pos->member != (head);                                            \
             pos = ulist_entry(pos->member.next, typeof(*pos), member))

#define ulist_for_each_entry_safe(pos, n, head, member)                       \
        for (pos = ulist_entry((head)->next, typeof(*pos), member),           \
               n = ulist_entry(pos->member.next, typeof(*pos), member);       \
             &pos->member != (head);                                            \
             pos = n, n = ulist_entry(n->member.next, typeof(*n), member))

#endif