/*******************************************************************************

  Summary:
  Simple command processor using serial line communication
    
  Description:
  This file contains a simple command processor parsing command lines
  received from a serial line.

  Note, this module is based on system_console, so SYS_CONSOLE_ENABLE must be defined to enable this.

*******************************************************************************/

/*******************************************************************************
FileName: 	system_command.c
Copyright ?2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/

#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include "system/system_services.h"
#include "system/system_command.h"
#include "system/system_debug.h"

#if defined(SYS_COMMAND_ENABLE)

/*
 * This command processor uses the simple serial console available in console.c
 Once you start the application it can send/receive messages using a hyper terminal
 connection.
 The Command Processor parses the commands received from the user
 and, upon recognition of the keyword, performs the associated commands.
*/



// definitions
#define 		MAX_CMD_GROUP	4
#define 		MAX_CMD_ARGS	15
#define         LINE_TERM       "\r\n"          // line terminator
#define         CMD_TOKEN_SEP   ", \r\n"        // command token separator
#define         _promptStr      ">"             // prompt string

#define         ESC_SEQ_SIZE    2               // standard VT100 escape sequences

#define         COMMAND_HISTORY_DEPTH   3


typedef struct _tagCmdNode
{
    struct _tagCmdNode* next;
    struct _tagCmdNode* prev;
    char    cmdBuff[_SYS_CMD_MAX_LENGTH+1];  // command itself
}cmdNode;   // simple command history   
    

typedef struct
{
    cmdNode*    head;
    cmdNode*    tail;
}cmdDlList;     // doubly linked command list

static _CMDIO_DEV_LST cmdIODevList = {0, 0, 0};

// data
static char				_cmdAlive = 0;

static _SYS_CMD_DCPT_TBL   _usrCmdTbl[MAX_CMD_GROUP] = { {0} };    // current command table

static cmdDlList        _cmdList = {0, 0};       // root of the command history

static cmdNode*         _pCurrCmdN;      // history pointer

static const char       _seqUpArrow[ESC_SEQ_SIZE] = "[A";
static const char       _seqDownArrow[ESC_SEQ_SIZE] = "[B";
static const char       _seqRightArrow[ESC_SEQ_SIZE] = "[C";
static const char       _seqLeftArrow[ESC_SEQ_SIZE] = "[D";  // standard VT100 escape sequences

// prototypes

static int 		CommandReset(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int      CommandQuit(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);              // command quit
static int      CommandHelp(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);              // help
static void     CommandCleanup(void);           // resources clean-up

static int 		StringToArgs(char *pRawString, char *argv[]); // Convert string to argc & argv[]
static int      ParseCmdBuffer(_CMDIO_DEV_NODE* pCmdIO);      // parse the command buffer


static void     ProcessEscSequence(_CMDIO_DEV_NODE* pCmdIO);       // process an escape sequence
//static void     DisplayPrompt(void);

static void     CmdAddHead(cmdDlList* pL, cmdNode* pN);
static cmdNode* CmdRemoveTail(cmdDlList* pL);

static void     DisplayNodeMsg(_CMDIO_DEV_NODE* pCmdIO, cmdNode* pNext);

#if defined(SYS_CONSOLE_ENABLE)
static void _sys_CONSOLE_MESSAGE(const void* cmdIoParam, const char* str);
static void _sys_CONSOLE_PRINT(const void* cmdIoParam, const char* format, ...);
static void _sys_CONSOLE_PUTC(const void* cmdIoParam, char c);
static bool _sys_CONSOLE_DATA_RDY(const void* cmdIoParam);
static char _sys_CONSOLE_GETC(const void* cmdIoParam);

const _CMDIO_DEV_API sysConsoleApi = 
{
	_sys_CONSOLE_MESSAGE,
	_sys_CONSOLE_PRINT,
	_sys_CONSOLE_PUTC,
	_sys_CONSOLE_DATA_RDY,
	_sys_CONSOLE_GETC,
};
#endif

// built-in command table
static const _SYS_CMD_DCPT    _builtinCmdTbl[]=
{
	{"reset", 	CommandReset, 	": Reset host"},
    {"q", 		CommandQuit, 	": quit command processor"},
    {"help",	CommandHelp,	": help"},
};



// interface functions 

bool _SYS_COMMAND_INIT(void)
{
    int         ix;

    CommandCleanup();       // just in case we have to deallocate previous data

    // construct the command history list
    for(ix = 0; ix<COMMAND_HISTORY_DEPTH; ix++)
    {
        cmdNode* pN;
        pN = (cmdNode*)SystemMalloc(sizeof(*pN));

        if(!pN)
        {
            return false;
        }
        pN->cmdBuff[0] = '\0';
        CmdAddHead(&_cmdList, pN);
    }
    _pCurrCmdN = 0;

// Console will be added automatically
#if defined(SYS_CONSOLE_ENABLE)
    // the console handle should be needed here but there's only one console for now
	_SYS_CMDIO_ADD(&sysConsoleApi, 0);
#endif
    
	//SYS_CONSOLE_MESSAGE(_promptStr);
	_cmdAlive = true;

    return true;
}

_CMDIO_DEV_NODE* _SYS_CMDIO_GET_HANDLE(short num)
{
    _CMDIO_DEV_NODE* pNode = cmdIODevList.head;
	
	if (num == 0) return ((pNode)?pNode:NULL);

    while(num && pNode)
    {
        pNode = pNode->next;
        num--;
    }

    return pNode;
}

_CMDIO_DEV_NODE* _SYS_CMDIO_ADD(const _CMDIO_DEV_API* opApi, const void* cmdIoParam)
{
	// Create node
	_CMDIO_DEV_NODE* pDevNode;
	pDevNode = (_CMDIO_DEV_NODE*)SystemMalloc(sizeof(*pDevNode));
	if (!pDevNode)
    {
        return 0;
	}
	pDevNode->pCmdApi = opApi;
	pDevNode->cmdIoParam = cmdIoParam;
	pDevNode->cmdPnt = pDevNode->cmdEnd = pDevNode->cmdBuff;
	//pDevNode->index = cmdIODevList.num;	

	// Insert node at end
	cmdIODevList.num++;
	    
    pDevNode->next = NULL;
    if(cmdIODevList.head == NULL)
    {
        cmdIODevList.head = pDevNode;
        cmdIODevList.tail = pDevNode;
    }
    else
    {
        cmdIODevList.tail->next = pDevNode;
        cmdIODevList.tail = pDevNode;
    } 
	
	return pDevNode;
}


bool _SYS_CMDIO_DELETE(_CMDIO_DEV_NODE* pDevNode)
{
    _CMDIO_DEV_NODE* p_listnode = cmdIODevList.head;
    _CMDIO_DEV_NODE* pre_listnode;
    
    //root list is empty or null node to be delete
    if((p_listnode==NULL) || (pDevNode==NULL))
    {
        return false;
    }
    
	// Head will be delted 
    if(p_listnode == pDevNode)
    {
        cmdIODevList.num--;
        //Root list has only one node
        if(cmdIODevList.tail == pDevNode)
        {
            cmdIODevList.head = NULL;
            cmdIODevList.tail = NULL;
        }
        else
        {
            cmdIODevList.head = p_listnode->next;
        }
		SystemFree(pDevNode);
        return true;
    }
	// Not head delete
	pre_listnode = p_listnode;
	while (p_listnode)
	{
		if(p_listnode == pDevNode)
    	{
        	pre_listnode->next = p_listnode->next;
    		// Deleted node is tail
			if (cmdIODevList.tail==pDevNode) {
				cmdIODevList.tail = pre_listnode;
			}
			cmdIODevList.num--;
			SystemFree(pDevNode);
        	return true;
    	}
        pre_listnode = p_listnode;
        p_listnode 	 = p_listnode->next;
	}

	return false;   
}


// add user command group
bool  _SYS_COMMAND_ADDGRP(const _SYS_CMD_DCPT* pCmdTbl, int nCmds, const char* groupName, const char* menuStr)
{
	int i, groupIx = -1, emptyIx = -1;
    int insertIx;

	// Check if there is space for new command group; If this table already added, also simply update. 
	for (i=0; i<MAX_CMD_GROUP; i++)
    {
        if(_usrCmdTbl[i].pCmd == 0)
        {   // empty slot
            emptyIx = i;
        }
        else if(_usrCmdTbl[i].pCmd == pCmdTbl)
        {   // already have this group; sanity check against the group name
            if(strcmp(groupName, _usrCmdTbl[i].cmdGroupName) != 0)
            {   // name mismatch
                return false;
            }

            groupIx = i;
            break;
        }
    }

	// reference the command group
	if (groupIx != -1)
    {
        insertIx = groupIx;
    }
    else if(emptyIx != -1)
    {
        insertIx = emptyIx;
    }
    else
    {
        return false;
    }

    _usrCmdTbl[insertIx].pCmd = pCmdTbl;
    _usrCmdTbl[insertIx].nCmds = nCmds;
    _usrCmdTbl[insertIx].cmdGroupName = groupName;
    _usrCmdTbl[insertIx].cmdMenuStr = menuStr;
    return true;

}

// processing command received by the serial line
bool _SYS_COMMAND_TASK(void)
{
	short i;
    char    data;
	_CMDIO_DEV_NODE* pCmdIO;
    const void*      cmdIoParam;

	if (_cmdAlive == false)
    {
       return false;
    }


	for (i=0; i<cmdIODevList.num; i++)
	{
		pCmdIO = _SYS_CMDIO_GET_HANDLE(i);
	    //DisplayPrompt();
	    
	    if(pCmdIO)
        {
            cmdIoParam = pCmdIO->cmdIoParam;

            if((*pCmdIO->pCmdApi->isRdy)(cmdIoParam))
            {   // some data available
                data = (*pCmdIO->pCmdApi->getc)(cmdIoParam); /* Read data from console. */
                if((data == '\r') || (data == '\n'))
                {   
                    // new command assembled
                    if(pCmdIO->cmdPnt ==  pCmdIO->cmdBuff)
                    {   // just an extra \n or \r
                        (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM _promptStr);
                        return true;
                    }
                    (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM);
                    *pCmdIO->cmdPnt = 0;
                    pCmdIO->cmdPnt = pCmdIO->cmdEnd = pCmdIO->cmdBuff;
                    ParseCmdBuffer(pCmdIO);
                    (*pCmdIO->pCmdApi->msg)(cmdIoParam, _promptStr);
                    return true;
                }
                else if(data == '\b')
                {
                    if(pCmdIO->cmdPnt > pCmdIO->cmdBuff)
                    {
                        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "\b \b");
                        pCmdIO->cmdPnt--; pCmdIO->cmdEnd--;
                    }
                }
                else if(data == 0x1b)
                { // escape sequence
                    ProcessEscSequence(pCmdIO);
                }   
                else if(pCmdIO->cmdEnd-pCmdIO->cmdBuff<_SYS_CMD_MAX_LENGTH)
                {
                    (*pCmdIO->pCmdApi->putc)(cmdIoParam, data);
                    *pCmdIO->cmdPnt++ = data;
                    if(pCmdIO->cmdPnt > pCmdIO->cmdEnd)
                    {
                        pCmdIO->cmdEnd = pCmdIO->cmdPnt;
                    }
                }
                else
                {
                    (*pCmdIO->pCmdApi->msg)(cmdIoParam, " *** Command Processor buffer exceeded. Retry. ***" LINE_TERM);
                    pCmdIO->cmdPnt = pCmdIO->cmdEnd = pCmdIO->cmdBuff;
                    (*pCmdIO->pCmdApi->msg)(cmdIoParam, _promptStr);
                }
            }
        }
	}
    return true;
}


#if defined(SYS_CONSOLE_ENABLE)

// ignore the console handle for now, we support a single system console
static void _sys_CONSOLE_MESSAGE(const void* cmdIoParam, const char* str)
{
    _SYS_CONSOLE_MESSAGE(str);
}

static void _sys_CONSOLE_PRINT(const void* cmdIoParam, const char* format, ...)
{
    char buff[_SYS_CMD_MAX_LENGTH];
    va_list args; 
    va_start( args, format ); 

    vsnprintf(buff, sizeof(buff), format, args);
    _SYS_CONSOLE_MESSAGE(buff);

    va_end( args );        

}


static void _sys_CONSOLE_PUTC(const void* cmdIoParam, char c)
{
    _SYS_CONSOLE_PUTC(c);
}


static bool _sys_CONSOLE_DATA_RDY(const void* cmdIoParam)
{
    return _SYS_CONSOLE_DATA_RDY();
}


static char _sys_CONSOLE_GETC(const void* cmdIoParam)
{
    return _SYS_CONSOLE_GETC();
}


#endif // defined(SYS_CONSOLE_ENABLE)

// implementation
static int CommandReset(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    const void* cmdIoParam = pCmdIO->cmdIoParam;
	(*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM " *** System Reboot ***\r\n" );
	SYS_Reboot(); 
	return 0;
}

// quit
static int CommandQuit(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
	_CMDIO_DEV_NODE* pCmdIoNode;
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM " *** Quitting the Command Processor. Bye ***\r\n" );

    CommandCleanup();

	while ((pCmdIoNode = _SYS_CMDIO_GET_HANDLE(0)) != NULL)
	{
		if(_SYS_CMDIO_DELETE(pCmdIoNode)) SystemFree(pCmdIoNode);
	}

    return 0;
}

// Remove command history list
static void CommandCleanup(void)
{
    cmdNode* pN;

    memset(_usrCmdTbl, 0x0, sizeof(_usrCmdTbl));

    while( (pN = CmdRemoveTail(&_cmdList)) )
    {
        SystemFree(pN);
    }

    _pCurrCmdN = 0;
	_cmdAlive = false;
}

static int CommandHelp(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    int             ix, groupIx;
    const _SYS_CMD_DCPT*  pDcpt;
    const _SYS_CMD_DCPT_TBL* pTbl, *pDTbl;
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if(argc == 1)
    {   // no params help; display basic info
        bool hadHeader = false;
        pTbl = _usrCmdTbl;
        for (groupIx=0; groupIx < MAX_CMD_GROUP; groupIx++)
        {
            if (pTbl->pCmd)
            {
                if(!hadHeader)
                {
                    (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM "------- Supported command groups ------");
                    hadHeader = true;
                }
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM " *** ");
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, pTbl->cmdGroupName);
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, pTbl->cmdMenuStr);
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, " ***");
            }
            pTbl++;
        }

        // display the basic commands
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM "---------- Built in commands ----------");
        for(ix = 0, pDcpt = _builtinCmdTbl; ix < sizeof(_builtinCmdTbl)/sizeof(*_builtinCmdTbl); ix++, pDcpt++)
        {
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM " *** ");
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, pDcpt->cmdStr);
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, pDcpt->cmdDescr);
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, " ***");
        }

        (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM);
    }
    else
    {   // we have a command group name
        pDTbl = 0;
        pTbl = _usrCmdTbl;
        for (groupIx=0; groupIx < MAX_CMD_GROUP; groupIx++)
        {
            if (pTbl->pCmd)
            {
                if(strcmp(pTbl->cmdGroupName, argv[1]) == 0)
                {   // match
                    pDTbl = pTbl;
                    break;
                }
            }
            pTbl++;
        }

        if(pDTbl)
        {
            for(ix = 0, pDcpt = pDTbl->pCmd; ix < pDTbl->nCmds; ix++, pDcpt++)
            {
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM " *** ");
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, pDcpt->cmdStr);
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, pDcpt->cmdDescr);
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, " ***");
            }

            (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM);
        }
        else
        {
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM "Unknown command group. Try help" LINE_TERM );
        }
    }
    return 1;

}

// dummy implementation
static int ParseCmdBuffer(_CMDIO_DEV_NODE* pCmdIO)
{
	int  argc;
    char *argv[MAX_CMD_ARGS + 1];
    static char saveCmd[_SYS_CMD_MAX_LENGTH+1];
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    int            ix, grp_ix;
    const _SYS_CMD_DCPT* pDcpt;
    
    strncpy(saveCmd, pCmdIO->cmdBuff, sizeof(saveCmd));     // make a copy of the command

	// parse a command string to *argv[] 
	argc = StringToArgs(saveCmd, argv);
	  
    if(argc != 0)
    {   // ok, there's smth here

        // add it to the history list
        cmdNode* pN = CmdRemoveTail(&_cmdList);
        strncpy(pN->cmdBuff, pCmdIO->cmdBuff, sizeof(saveCmd));	// Need save non-parsed string
        CmdAddHead(&_cmdList, pN);
        _pCurrCmdN = 0;
        
        // try built-in commands first
        for(ix = 0, pDcpt = _builtinCmdTbl; ix < sizeof(_builtinCmdTbl)/sizeof(*_builtinCmdTbl); ix++, pDcpt++)
        {
            if(!strcmp(argv[0], pDcpt->cmdStr))
            {   // command found
                return (*pDcpt->cmdFnc)(pCmdIO, argc, argv);     // call command handler
            }
        }
        // search user commands
        for (grp_ix=0; grp_ix<MAX_CMD_GROUP; grp_ix++)
        {		
			if (_usrCmdTbl[grp_ix].pCmd == 0)
            {
               continue;
            }

		    for(ix = 0, pDcpt = _usrCmdTbl[grp_ix].pCmd; ix < _usrCmdTbl[grp_ix].nCmds; ix++, pDcpt++)
		    {
		        if(!strcmp(argv[0], pDcpt->cmdStr))
		        {   // command found
		            return (*pDcpt->cmdFnc)(pCmdIO, argc, argv); // call command handler
		        }
		    }
		}
        
        // command not found
        (*pCmdIO->pCmdApi->print)(cmdIoParam, " *** Command Processor: unknown command: %s ***\r\n", argv[0]);
    }
    else
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, " *** Command Processor: Please type in a command***" LINE_TERM);
    }
    
    return 1;
}

/*
  parse a tring into '*argv[]', delimitor is space or tab
  param pRawString, the whole line of command string
  param argv, parsed argument string array 
  return number of parsed argument
*/
static int StringToArgs(char *pRawString, char *argv[]) {
  int argc = 0, i = 0, strsize = 0;
  
  if(pRawString == NULL)
    return 0;

  strsize = strlen(pRawString);
  
  while(argc < MAX_CMD_ARGS) {
    
    // skip white space characters of string head 
    while ((*pRawString == ' ') || (*pRawString == '\t')) {
      ++pRawString;
	  if (++i >= strsize) {
		return (argc);		
	  }
    }
    
    if (*pRawString == '\0') {
      argv[argc] = NULL;
      return (argc);
    }
    
    argv[argc++] = pRawString;
    
    // find end of string 
    while (*pRawString && (*pRawString != ' ') && (*pRawString != '\t')) {
      ++pRawString;
    }

    if (*pRawString == '\0') {
    argv[argc] = NULL;
    return (argc);
    }
    
    *pRawString++ = '\0';
  }
  
  SYS_CONSOLE_PRINT("\n\r Too many arguments. Maximum argus supported is %d!\r\n", MAX_CMD_ARGS);
  
  return (0);
}


static void ProcessEscSequence(_CMDIO_DEV_NODE* pCmdIO)
{
    cmdNode *pNext;
    char    seq[ESC_SEQ_SIZE+1];
    int nChar = 0;
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    while(nChar<ESC_SEQ_SIZE)
    {
        while(!(*pCmdIO->pCmdApi->isRdy)(cmdIoParam));
        seq[nChar++] = (*pCmdIO->pCmdApi->getc)(cmdIoParam);
    }
    seq[nChar] = '\0';

    if(!strcmp(seq, _seqUpArrow))
    { // up arrow
        if(_pCurrCmdN)
        {
            pNext = _pCurrCmdN->next;
            if(pNext == _cmdList.head)
            {
                return; // reached the end of list
            }
        }
        else
        {
            pNext = _cmdList.head;
        }
        
        DisplayNodeMsg(pCmdIO, pNext);
    }
    else if(!strcmp(seq, _seqDownArrow))
    { // down arrow
        if(_pCurrCmdN)
        {
            pNext = _pCurrCmdN->prev;
            if(pNext != _cmdList.tail)
            {
                DisplayNodeMsg(pCmdIO, pNext);
            }
        }
    }
    else if(!strcmp(seq, _seqRightArrow))
    { // right arrow
        if(pCmdIO->cmdPnt < pCmdIO->cmdEnd)
        {
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, _seqRightArrow);   
            pCmdIO->cmdPnt++;
        }
    }   
    else if(!strcmp(seq, _seqLeftArrow))
    { // left arrow
        if(pCmdIO->cmdPnt > pCmdIO->cmdBuff)
        {
            pCmdIO->cmdPnt--;
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, _seqLeftArrow);
        }
    }   
    else
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, " *** Command Processor: unknown command. ***" LINE_TERM);
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, _promptStr);
    }
    
}

static void DisplayNodeMsg(_CMDIO_DEV_NODE* pCmdIO, cmdNode* pNext)
{
    int oCmdLen, nCmdLen;
    const void* cmdIoParam = pCmdIO->cmdIoParam;
    
    if((nCmdLen = strlen(pNext->cmdBuff)))
    {   // something there	
        oCmdLen = pCmdIO->cmdEnd-pCmdIO->cmdBuff;
        while(oCmdLen>nCmdLen)
        {
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "\b \b");     // clear the old command
            oCmdLen--;
        }
        while(oCmdLen--)
        {
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "\b");            
        }
        strcpy(pCmdIO->cmdBuff, pNext->cmdBuff);
		(*pCmdIO->pCmdApi->msg)(cmdIoParam, "\r\n>");
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, pCmdIO->cmdBuff);
        pCmdIO->cmdPnt = pCmdIO->cmdEnd = pCmdIO->cmdBuff+nCmdLen;
        _pCurrCmdN = pNext;
    }
}


static void CmdAddHead(cmdDlList* pL, cmdNode* pN)
{
    if(pL->head == 0)
    { // empty list, first node
        pL->head = pL->tail = pN;
        pN->next = pN->prev = pN;
    }
    else
    {
        pN->next = pL->head;
        pN->prev = pL->tail;
        pL->tail->next = pN;
        pL->head->prev = pN;
        pL->head = pN;
    }       
}


static cmdNode* CmdRemoveTail(cmdDlList* pL)
{
    cmdNode* pN;
    if(pL->head == pL->tail)
    {
        pN = pL->head;
        pL->head = pL->tail = 0;
    }
    else
    {
        pN = pL->tail;
        pL->tail = pN->prev;
        pL->tail->next = pL->head;
        pL->head->prev = pL->tail;
    }
    return pN;
}

#endif


