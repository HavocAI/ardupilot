/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

template<typename Context>
class BaseState
{
public:
    virtual ~BaseState() = default;

    // Called once when the state is entered
    virtual void enter(Context &ctx) = 0;

    // Called on each update cycle (e.g. in the main loop)
    virtual void run(Context &ctx) = 0;

    // Called once when the state is exited
    virtual void exit(Context &ctx) = 0;
};
