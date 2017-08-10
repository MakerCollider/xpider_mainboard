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

#include "task.h"

Task::Task() {
	m_pfnTask = 0;
	m_interval = TASK_DEFAULT_INTERVAL;
	m_lastTrigger = 0;
	m_enabled = false;
}
void Task::init(unsigned long us, task_func pfn) {
	if(us<TASK_MIN_INTERVAL || pfn==0) {
		m_enabled = false;
		return;
	}
	m_interval = us;
	m_pfnTask = pfn;
	m_enabled = true;
}

void Task::trigger(unsigned long currentUs) {
	if(!m_enabled) {
		return;
	}
	if(currentUs-m_lastTrigger > m_interval) {
		if(m_pfnTask) {
			(*m_pfnTask)();
		}
		m_lastTrigger = currentUs;
	}
}

bool Task::enabled() {
	return m_enabled;
}

void Task::setEnabled(bool b) {
	m_enabled = b;
}


