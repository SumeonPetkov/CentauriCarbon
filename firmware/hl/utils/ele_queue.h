/**
 * @file mem_pool.h
 * @brief cbd queue
 * @version 1.0
 * @date 2024-05-30
 * 
 */
#ifndef _ELE_QUEUE_H_
#define _ELE_QUEUE_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include "cbd_types.h"
#include "cbd_error_code.h"
typedef VOID_T* CBD_QUEUE_HANDLE;
typedef BOOL_T (*TRAVERSE_CB)(VOID_T*item, VOID_T *ctx);

/**
 * @brief create and initialize a queue (FIFO)
 * 
 * @param[in] queue_len the maximum number of items that the queue can contain.
 * @param[in] item_size the number of bytes each item in the queue will require.
 * @param[out] handle the queue handle
 * 
 * @note items are queued by copy, not by reference. Each item on the queue must be the same size.
 *
 * @return OPRT_OK on success. Others on error, please refer to cbd_error_code.h
 */
CBD_ERROR_CODE cbd_queue_create(CONST UINT32_T queue_len, CONST UINT32_T item_size, CBD_QUEUE_HANDLE *handle);

/**
 * @brief enqueue, append to the tail
 *
 * @param[in] handle the queue handle
 * @param[in] item pointer to the item that is to be placed on the queue.
 *
 * @return OPRT_OK on success. Others on error, please refer to cbd_error_code.h
 */
CBD_ERROR_CODE cbd_queue_input(CBD_QUEUE_HANDLE handle, CONST VOID_T *item);

/**
 * @brief enqueue, insert to the head
 *
 * @param[in] handle the queue handle
 * @param[in] item pointer to the item that is to be placed on the queue.
 *
 * @return OPRT_OK on success. Others on error, please refer to cbd_error_code.h
 */
CBD_ERROR_CODE cbd_queue_input_instant(CBD_QUEUE_HANDLE handle, CONST VOID_T *item);

/**
 * @brief dequeue
 *
 * @param[in] handle the queue handle
 * @param[in] item the dequeue item buffer, NULL indicates discard the item
 *
 * @return OPRT_OK on success, others on failed, please refer to cbd_error_code.h
 */
CBD_ERROR_CODE cbd_queue_output(CBD_QUEUE_HANDLE handle, CONST VOID_T *item);

/**
 * @brief get the peek item(not dequeue)
 *
 * @param[in] handle the queue handle
 * @param[out] item pointer to the buffer into which the received item will be copied.
 *
 * @return OPRT_OK on success. Others on error, please refer to cbd_error_code.h
 */
CBD_ERROR_CODE cbd_queue_peek(CBD_QUEUE_HANDLE handle, CONST VOID_T *item);

/**
 * @brief traverse the queue with specific callback
 *
 * @param[in] handle the queue handle
 * @param[in] cb the callback
 * @param[in] ctx the callback context
 *
 * @return OPRT_OK on success. Others on error, please refer to cbd_error_code.h 
 */
CBD_ERROR_CODE cbd_queue_traverse(CBD_QUEUE_HANDLE handle, TRAVERSE_CB cb, VOID_T *ctx);

/**
 * @brief clear all items in the queue
 *
 * @param[in] handle the queue handle
 *
 * @return OPRT_OK on success. Others on error, please refer to cbd_error_code.h 
 */
CBD_ERROR_CODE cbd_queue_clear(CBD_QUEUE_HANDLE handle);

/**
 * @brief get items from start postion, not dequeue
 *
 * @param[in] handle the queue handle
 * @param[in] start the start postion
 * @param[in] items the item buffer
 * @param[in] num the item counts
 *
 * @return OPRT_OK on success. Others on error, please refer to cbd_error_code.h
 */
CBD_ERROR_CODE cbd_queue_get_batch(CBD_QUEUE_HANDLE handle, CONST UINT32_T start, VOID_T *items, CONST UINT32_T num);

/**
 * @brief delete the item from the queue position
 *
 * @param[in] handle the queue handle
 * @param[in] num the item count to be deleted from the queue
 *
 * @return OPRT_OK on success. Others on error, please refer to cbd_error_code.h
 */
CBD_ERROR_CODE cbd_queue_delete_batch(CBD_QUEUE_HANDLE handle, CONST UINT32_T num);

/**
 * @brief get the free queue item number
 *
 * @param[in] handle the queue handle
 *
 * @return the current free item counts
 */
UINT32_T cbd_queue_get_free_num(CBD_QUEUE_HANDLE handle);

/**
 * @brief get the queue item number
 *
 * @param[in] handle the queue handle
 *
 * @return the current item counts 
 */
UINT32_T cbd_queue_get_used_num(CBD_QUEUE_HANDLE handle);

/**
 * @brief get the queue item number
 *
 * @param[in] handle the queue handle
 *
 * @return the current item counts 
 */
UINT32_T cbd_queue_get_max_num(CBD_QUEUE_HANDLE handle);

/**
 * @brief release the queue
 *
 * @param[in] handle the queue handle
 *
 * @return OPRT_OK on success. Others on error, please refer to cbd_error_code.h
 */
CBD_ERROR_CODE cbd_queue_release(CBD_QUEUE_HANDLE handle);

#ifdef __cplusplus
}
#endif

#endif