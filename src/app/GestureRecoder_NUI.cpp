/*
 * This file is part of the CAR-NUI project.
 * Copyright (C) 2012 DFKI GmbH. All rights reserved.
 *
 * Disclaimer:
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND
 * CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "core/common.hpp"
#include "eventmanager/NUIEventManager.hpp"
#include <iostream>
#include <limits>
//#include <gflags/gflags.h>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>


void wait() {
	std::cout << "Press ENTER to stop NUI...";

}

int main(int argc, char *argv[]) {
	//Start the singleton eventmanager
	nui::configname("gesture-recorder");
	NUIEventManager::instance()->start();
	char key = ' ';
	while ('q' != key) {
		wait();
		key = getchar();
	}
	NUIEventManager::instance()->stop();
	return 0;
}
