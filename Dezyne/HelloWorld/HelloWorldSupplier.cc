// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "HelloWorldSupplier.hh"

#include "locator.hh"
#include "runtime.hh"

#include <iostream>
#include <string>
#include <functional>


void set_hello_world(std::string& value) {
	value = "Hello world!";
}

std::function<void (std::string&)> f_set_hello_world = set_hello_world;
