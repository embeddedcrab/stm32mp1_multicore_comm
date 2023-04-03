/*********************************************************************************************************************
 * @file     
 * @brief    
 * @version  1.0.0
 * @date	 03/08/2023
 *
 * @cond
 *********************************************************************************************************************
 * Copyright (c) 2023, EmbeddedCrab (Hemant Sharma) - All Rights Reserved
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************************************************
 *
 * *************************** Change history ********************************
 *
 * @endcond
 */
/******************************************************************************
* Notes:
*
* Change History
* --------------
*
*******************************************************************************/

/** @file:	
 *  @brief:	This file contains.
 */
#ifndef _MESSAGE_CONVERSION_HPP_
#define _MESSAGE_CONVERSION_HPP_


/******************************************************************************
* Includes
******************************************************************************/
#include <string>

#include <iostream>
#include <sstream>


/******************************************************************************
* Configuration Constants
******************************************************************************/


/******************************************************************************
* Class Declarations/Definition
******************************************************************************/

// communication namespace
namespace comm
{

/**
 * Serializer for Communication Data, uses a fixed format
 */
void SerializeCommData( std::string const &in, const uint32_t size, std::string &out)
{
  std::ostringstream oss{};
  // Form string from input data and update it into string var
  oss << in << "," << size;
  out = oss.str();
}

void SerializeCommData( std::string const &name, const uint32_t len, const uint32_t id, std::string &out)
{
  std::ostringstream oss{};
  // Form string from input data and update it into string var
  oss << len << ',' << id << ',' << name;
  out = oss.str();
}
void DeserializeCommDataCreate(std::string in, uint32_t &size, uint32_t &id, std::string &name)
{
  std::istringstream stream{in};
  char ch{};

  stream >> size >> ch >> id >> ch >> name;
}

void SerializeCommData( const uint32_t id, const uint32_t size, std::string &out)
{
  std::ostringstream oss{};
  // Form string from input data and update it into string var
  oss << size << "," << id;
  out = oss.str();
}

void DeserializeCommDataCreate(std::string in, uint32_t &id, uint32_t &size)
{
  // Extract value before delimiter
  std::string::size_type pos = in.find(',');
  id = std::stoi(in.substr(0, pos));
  // Update value after delimited in size
  size = std::stoi(in.substr(pos + 1));
}

/**
 * De-Serializer for Communication Data, uses a fixed format
 */
void DeserializeCommData(std::string const& in, std::string &out, uint32_t &size)
{
  // Extract value before delimiter
  std::string::size_type pos = in.find(',');
  out = in.substr(0, pos);
  // Update value after delimited in size
  size = std::stoi(in.substr(pos + 1));
}

/**
 * De-Serializer for Communication Data, uses a fixed format
 * 
 * Accepts raw string data as input and create string locally
 */
void DeserializeCommDataCreate(std::string in, std::string &out, uint32_t &size)
{
  // Extract value before delimiter
  std::string::size_type pos = in.find(',');
  out = in.substr(0, pos);
  // Update value after delimited in size
  size = std::stoi(in.substr(pos + 1));
}

} // namespace comm

#endif  // message_conversion.hpp

/********************************** End of File *******************************/