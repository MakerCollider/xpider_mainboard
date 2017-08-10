/**
* Author: Yunpeng Song <413740951@qq.com>
* Copyright (c) 2016 Maker Collider Corporation.
*
* Permission is hereby granted, free of charge, to any person obtaining
* a copy of this software and associated documentation files (the
* "Software"), to deal in the Software without restriction, including
* without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to
* permit persons to whom the Software is furnished to do so, subject to
* the following conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*/

#ifndef TASK_H_
#define TASK_H_

#define TASK_MIN_INTERVAL 5000        //2ms
#define TASK_DEFAULT_INTERVAL 50000   //50ms

class Task {
public:
  typedef void (*task_func)();
  
	Task();
 
	void init(unsigned long us, task_func pfn);
	void trigger(unsigned long currentUs);
	bool enabled();
	void setEnabled(bool b);
protected:
  bool m_enabled;
	unsigned long m_interval;
	unsigned long m_lastTrigger;

  task_func m_pfnTask;
};

#endif // TASK_H_


