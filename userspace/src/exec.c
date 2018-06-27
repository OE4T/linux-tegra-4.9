/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <stdlib.h>

#include <unit/io.h>
#include <unit/core.h>
#include <unit/unit.h>
#include <unit/module.h>
#include <unit/results.h>

#include <nvgpu/posix/probe.h>

/*
 * Execute a module and all its subtests. This function builds a gk20a for the
 * test to use by executing nvgpu_posix_probe() and nvgpu_posix_cleanup();
 */
static int core_exec_module(struct unit_fw *fw,
			    struct unit_module *module)
{
	unsigned int i;
	struct gk20a *g = fw->nvgpu.nvgpu_posix_probe();

	if (!g)
		return -1;

	core_vbs(fw, 1, "Execing module: %s\n", module->name);

	/*
	 * Execute each test within the module. No reinit is done between tests.
	 * Thats up to the module itself to handle. Any setup/teardown between
	 * unit tests must be handled within the module.
	 */
	for (i = 0; i < module->nr_tests; i++) {
		struct unit_module_test *t = module->tests + i;
		int test_status;

		core_msg(fw, "Running %s.%s\n", module->name, t->name);
		test_status = t->fn(module, g, t->args);

		if (test_status != UNIT_SUCCESS)
			core_msg_color(fw, C_RED,
				       "  Unit error! Test %s.%s FAILED!\n",
				       module->name, t->name);

		core_add_test_record(fw, module, t,
				     test_status == UNIT_SUCCESS);
	}

	fw->nvgpu.nvgpu_posix_cleanup(g);

	return 0;
}

/*
 * Execute all modules loaded by the unit test framework.
 */
int core_exec(struct unit_fw *fw)
{
	int ret;
	struct unit_module **modules;

	for (modules = fw->modules; *modules != NULL; modules++) {
		ret = core_exec_module(fw, *modules);

		if (ret != 0)
			return ret;
	}

	return 0;
}
