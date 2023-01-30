pxReadyTasksLists[n] is a linked list of FreeRTOS internal type "List_t" holding ready tasks of priority n.

pxReadyTaskLists[n] holds tasks that can run immediately. It's of type List_t (defined in FreeRTOS/Source/include/list.h).

member pxIndex of List_t points to an element in the list. Which element it points to changes as the list is traversed. It seems like there's a dummy element that it normally points to before the list is traversed, so to reach the first element in the list, you need to go to pxIndex->pxNext.

pvOwner is a void*, but it's typically a TCB_t*.

For instance, to print the 0th item in the list, you'd do
    p/x *(TCB_t*)(pxReadyTasksLists[n].pxIndex->pxNext->pvOwner)
or
    p/x *(TCB_t*)(xDelayedTaskList1.pxIndex->pxNext->pvOwner)

To print the 2nd one, you'd do
    p/x *(TCB_t*)(pxReadyTasksLists[n].pxIndex->pxNext->pxNext->pxNext->pvOwner)

To get its name, you'd do
    p ((TCB_t*)(xDelayedTaskList1.pxIndex->pxNext->pvOwner))->pcTaskName


To get the return address for a task, you'd first look at names to find the TCB
    p ((TCB_t*)(xDelayedTaskList1.pxIndex->pxNext->pvOwner))->pcTaskName
        "defaultTask\000\000\000\000"
    p ((TCB_t*)(xDelayedTaskList1.pxIndex->pxNext->pxNext->pvOwner))->pcTaskName
        "annotator task\000"
    p ((TCB_t*)(xDelayedTaskList1.pxIndex->pxNext->pxNext->pxNext->pvOwner))->pcTaskName
        "indic task\000\000\000\000\000"

^^^ let's say that's the one we want.
The TCB_t struct is defined around line 250 of tasks.c as "tskTaskControlBlock"

    p/x *(TCB_t*)(xDelayedTaskList1.pxIndex->pxNext->pxNext->pxNext->pvOwner)

There are a few places we can find tasks:
    pxReadyTasksLists[n] (for priority n)
    xDelayedTaskList1
    xDelayedTaskList2
    xPendingReadyList
    xSuspendedTaskList

Get return address from SVC_Handler
p/x ((uint32_t*)$psp)[6]
