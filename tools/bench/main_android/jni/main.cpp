////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2019 Nicholas Frechette & Realtime Math contributors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
////////////////////////////////////////////////////////////////////////////////

#include <benchmark/benchmark.h>

#include <android/log.h>
#include <iostream>
#include <jni.h>
#include <streambuf>

// Inspired from https://stackoverflow.com/questions/8870174/is-stdcout-usable-in-android-ndk
class androidbuf final : public std::streambuf
{
public:
	enum { bufsize = 4096 };
	androidbuf()
	{
		this->setp(buffer, buffer + bufsize - 1);
	}

private:
	int overflow(int c)
	{
		if (c == traits_type::eof())
		{
			*this->pptr() = traits_type::to_char_type(c);
			this->sbumpc();
		}
		return this->sync() ? traits_type::eof() : traits_type::not_eof(c);
	}

	int sync()
	{
		int rc = 0;
		if (this->pbase() != this->pptr())
		{
			char writebuf[bufsize + 1];
			memcpy(writebuf, this->pbase(), this->pptr() - this->pbase());
			writebuf[this->pptr() - this->pbase()] = '\0';

			rc = __android_log_write(ANDROID_LOG_INFO, "rtm", writebuf) > 0;
			this->setp(buffer, buffer + bufsize - 1);
		}
		return rc;
	}

	char buffer[bufsize];
};

extern "C" jint Java_com_rtm_benchmark_MainActivity_runBenchmark(JNIEnv* env, jobject caller)
{
	std::cout.rdbuf(new androidbuf());

	int argc = 0;
	benchmark::Initialize(&argc, nullptr);

	benchmark::RunSpecifiedBenchmarks();

	return 0;
}
