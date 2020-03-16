/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo */

/*******************************************************************************
 * Modified by Michael Dodson, 2020
*******************************************************************************/

#include "turtlebot3_rosserial.h"
#include "simulation_parameters.h"

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{

  std::string cl_svr_ip="128.232.65.230";
  std::string host_ip;

  if(argc==1)
  {
    host_ip = cl_svr_ip;
  }
  else if(argc==2)
  {
    host_ip=argv[1];
  }
  else
  {
    printf("Usage: turtlebot3_rosserial [host_ip]\n");
    return 1;
  }

  int count = 0;
  int spin_result;

  initPhysical();
  initComms(host_ip);

  while (1)
  {
    updatePhysical();
    updateComms();

    spin_result=spinComms();

    if(count%10 == 0)
    {
      printf("Loop count: %d\n", count);
    }
    count++;

    sleep(loop_rate);
  }

  return 0;
}