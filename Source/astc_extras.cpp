/*----------------------------------------------------------------------------*/
/**
 *	This confidential and proprietary software may be used only as
 *	authorised by a licensing agreement from ARM Limited
 *	(C) COPYRIGHT 2011-2013 ARM Limited
 *	ALL RIGHTS RESERVED
 *
 *	The entire notice above must be reproduced on all authorised
 *	copies and copies may only be made to the extent permitted
 *	by a licensing agreement from ARM Limited.
 *
 *	@brief	Top level functions - parsing command line, managing conversions,
 *			etc.
 *
 *			This is also where main() lives.
 */
/*----------------------------------------------------------------------------*/

#include "astc_codec_internals.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef WIN32
	#include <sys/time.h>
	#include <pthread.h>
	#include <unistd.h>

	double get_time()
	{
		timeval tv;
		gettimeofday(&tv, 0);

		return (double)tv.tv_sec + (double)tv.tv_usec * 1.0e-6;
	}


	int astc_codec_unlink(const char *filename)
	{
		return unlink(filename);
	}

#else
	// Windows.h defines IGNORE, so we must #undef our own version.
	#undef IGNORE

	// Define pthread-like functions in terms of Windows threading API
	#define WIN32_LEAN_AND_MEAN
	#include <windows.h>

	typedef HANDLE pthread_t;
	typedef int pthread_attr_t;

	int pthread_create(pthread_t * thread, const pthread_attr_t * attribs, void *(*threadfunc) (void *), void *thread_arg)
	{
		*thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) threadfunc, thread_arg, 0, NULL);
		return 0;
	}

	int pthread_join(pthread_t thread, void **value)
	{
		WaitForSingleObject(thread, INFINITE);
		return 0;
	}

	double get_time()
	{
		FILETIME tv;
		GetSystemTimeAsFileTime(&tv);

		unsigned __int64 ticks = tv.dwHighDateTime;
		ticks = (ticks << 32) | tv.dwLowDateTime;

		return ((double)ticks) / 1.0e7;
	}

	// Define an unlink() function in terms of the Win32 DeleteFile function.
	int astc_codec_unlink(const char *filename)
	{
		BOOL res = DeleteFileA(filename);
		return (res ? 0 : -1);
	}
#endif

#ifdef DEBUG_CAPTURE_NAN
	#ifndef _GNU_SOURCE
		#define _GNU_SOURCE
	#endif

	#include <fenv.h>
#endif

// Define this to be 1 to allow "illegal" block sizes
#define DEBUG_ALLOW_ILLEGAL_BLOCK_SIZES 0

extern int block_mode_histogram[2048];

#ifdef DEBUG_PRINT_DIAGNOSTICS
	int print_diagnostics = 0;
	int diagnostics_tile = -1;
#endif

int print_tile_errors = 0;

int print_statistics = 0;

int progress_counter_divider = 1;

int rgb_force_use_of_hdr = 0;
int alpha_force_use_of_hdr = 0;


static double start_time;
static double end_time;
static double start_coding_time;
static double end_coding_time;


// code to discover the number of logical CPUs available.

#if defined(__APPLE__)
	#define _DARWIN_C_SOURCE
	#include <sys/types.h>
	#include <sys/sysctl.h>
#endif

#if defined(_WIN32) || defined(__CYGWIN__)
	#include <windows.h>
#else
	#include <unistd.h>
#endif



unsigned get_number_of_cpus(void)
{
	unsigned n_cpus = 1;

	#ifdef __linux__
		cpu_set_t mask;
		CPU_ZERO(&mask);
		sched_getaffinity(getpid(), sizeof(mask), &mask);
		n_cpus = 0;
		for (unsigned i = 0; i < CPU_SETSIZE; ++i)
		{
			if (CPU_ISSET(i, &mask))
				n_cpus++;
		}
		if (n_cpus == 0)
			n_cpus = 1;

	#elif defined (_WIN32) || defined(__CYGWIN__)
		SYSTEM_INFO sysinfo;
		GetSystemInfo(&sysinfo);
		n_cpus = sysinfo.dwNumberOfProcessors;

	#elif defined(__APPLE__)
		int mib[4];
		size_t length = 100;
		mib[0] = CTL_HW;
		mib[1] = HW_AVAILCPU;
		sysctl(mib, 2, &n_cpus, &length, NULL, 0);
	#endif

	return n_cpus;
}

void astc_codec_internal_error(const char *filename, int linenum)
{
	printf("Internal error: File=%s Line=%d\n", filename, linenum);
	exit(1);
}

#define MAGIC_FILE_CONSTANT 0x5CA1AB13

struct astc_header
{
	uint8_t magic[4];
	uint8_t blockdim_x;
	uint8_t blockdim_y;
	uint8_t blockdim_z;
	uint8_t xsize[3];			// x-size = xsize[0] + xsize[1] + xsize[2]
	uint8_t ysize[3];			// x-size, y-size and z-size are given in texels;
	uint8_t zsize[3];			// block count is inferred
};


int suppress_progress_counter = 0;
int perform_srgb_transform = 0;

astc_codec_image *load_astc_file(const char *filename, int bitness, astc_decode_mode decode_mode, swizzlepattern swz_decode)
{
	int x, y, z;
	FILE *f = fopen(filename, "rb");
	if (!f)
	{
		printf("Failed to open file %s\n", filename);
		exit(1);
	}
	astc_header hdr;
	size_t hdr_bytes_read = fread(&hdr, 1, sizeof(astc_header), f);
	if (hdr_bytes_read != sizeof(astc_header))
	{
		fclose(f);
		printf("Failed to read file %s\n", filename);
		exit(1);
	}

	uint32_t magicval = hdr.magic[0] + 256 * (uint32_t) (hdr.magic[1]) + 65536 * (uint32_t) (hdr.magic[2]) + 16777216 * (uint32_t) (hdr.magic[3]);

	if (magicval != MAGIC_FILE_CONSTANT)
	{
		fclose(f);
		printf("File %s not recognized\n", filename);
		exit(1);
	}

	int xdim = hdr.blockdim_x;
	int ydim = hdr.blockdim_y;
	int zdim = hdr.blockdim_z;

	if ( (xdim < 3 || xdim > 6 || ydim < 3 || ydim > 6 || zdim < 3 || zdim > 6) &&
	     (xdim < 4 || xdim == 7 || xdim == 9 || xdim == 11 || xdim > 12 ||
	      ydim < 4 || ydim == 7 || ydim == 9 || ydim == 11 || ydim > 12 || zdim != 1) )
	{
		fclose(f);
		printf("File %s not recognized %d %d %d\n", filename, xdim, ydim, zdim);
		exit(1);
	}


	int xsize = hdr.xsize[0] + 256 * hdr.xsize[1] + 65536 * hdr.xsize[2];
	int ysize = hdr.ysize[0] + 256 * hdr.ysize[1] + 65536 * hdr.ysize[2];
	int zsize = hdr.zsize[0] + 256 * hdr.zsize[1] + 65536 * hdr.zsize[2];

	if (xsize == 0 || ysize == 0 || zsize == 0)
	{
		fclose(f);
		printf("File %s has zero dimension %d %d %d\n", filename, xsize, ysize, zsize);
		exit(1);
	}


	int xblocks = (xsize + xdim - 1) / xdim;
	int yblocks = (ysize + ydim - 1) / ydim;
	int zblocks = (zsize + zdim - 1) / zdim;

	uint8_t *buffer = (uint8_t *) malloc(xblocks * yblocks * zblocks * 16);
	if (!buffer)
	{
		fclose(f);
		printf("Ran out of memory\n");
		exit(1);
	}
	size_t bytes_to_read = xblocks * yblocks * zblocks * 16;
	size_t bytes_read = fread(buffer, 1, bytes_to_read, f);
	fclose(f);
	if (bytes_read != bytes_to_read)
	{
		printf("Failed to read file %s\n", filename);
		exit(1);
	}


	astc_codec_image *img = allocate_image(bitness, xsize, ysize, zsize, 0);
	initialize_image(img);

	imageblock pb;
	for (z = 0; z < zblocks; z++)
		for (y = 0; y < yblocks; y++)
			for (x = 0; x < xblocks; x++)
			{
				int offset = (((z * yblocks + y) * xblocks) + x) * 16;
				uint8_t *bp = buffer + offset;
				physical_compressed_block pcb = *(physical_compressed_block *) bp;
				symbolic_compressed_block scb;
				physical_to_symbolic(xdim, ydim, zdim, pcb, &scb);
				decompress_symbolic_block(decode_mode, xdim, ydim, zdim, x * xdim, y * ydim, z * zdim, &scb, &pb);
				write_imageblock(img, &pb, xdim, ydim, zdim, x * xdim, y * ydim, z * zdim, swz_decode);
			}

	free(buffer);

	return img;
}



struct encode_astc_image_info
{
	int xdim;
	int ydim;
	int zdim;
	const error_weighting_params *ewp;
	uint8_t *buffer;
	int *counters;
	int pack_and_unpack;
	int thread_id;
	int threadcount;
	astc_decode_mode decode_mode;
	swizzlepattern swz_encode;
	swizzlepattern swz_decode;
	int *threads_completed;
	const astc_codec_image *input_image;
	astc_codec_image *output_image;
};



void *encode_astc_image_threadfunc(void *vblk)
{
	const encode_astc_image_info *blk = (const encode_astc_image_info *)vblk;
	int xdim = blk->xdim;
	int ydim = blk->ydim;
	int zdim = blk->zdim;
	uint8_t *buffer = blk->buffer;
	const error_weighting_params *ewp = blk->ewp;
	int thread_id = blk->thread_id;
	int threadcount = blk->threadcount;
	int *counters = blk->counters;
	int pack_and_unpack = blk->pack_and_unpack;
	astc_decode_mode decode_mode = blk->decode_mode;
	swizzlepattern swz_encode = blk->swz_encode;
	swizzlepattern swz_decode = blk->swz_decode;
	int *threads_completed = blk->threads_completed;
	const astc_codec_image *input_image = blk->input_image;
	astc_codec_image *output_image = blk->output_image;

	imageblock pb;
	int ctr = thread_id;
	int pctr = 0;

	int x, y, z, i;
	int xsize = input_image->xsize;
	int ysize = input_image->ysize;
	int zsize = input_image->zsize;
	int xblocks = (xsize + xdim - 1) / xdim;
	int yblocks = (ysize + ydim - 1) / ydim;
	int zblocks = (zsize + zdim - 1) / zdim;

	int owns_progress_counter = 0;

	//allocate memory for temporary buffers
	compress_symbolic_block_buffers temp_buffers;
	temp_buffers.ewb = new error_weight_block;
	temp_buffers.ewbo = new error_weight_block_orig;
	temp_buffers.tempblocks = new symbolic_compressed_block[4];
	temp_buffers.temp = new imageblock;
	temp_buffers.planes2 = new compress_fixed_partition_buffers;
	temp_buffers.planes2->ei1 = new endpoints_and_weights;
	temp_buffers.planes2->ei2 = new endpoints_and_weights;
	temp_buffers.planes2->eix1 = new endpoints_and_weights[MAX_DECIMATION_MODES];
	temp_buffers.planes2->eix2 = new endpoints_and_weights[MAX_DECIMATION_MODES];
	temp_buffers.planes2->decimated_quantized_weights = new float[2 * MAX_DECIMATION_MODES * MAX_WEIGHTS_PER_BLOCK];
	temp_buffers.planes2->decimated_weights = new float[2 * MAX_DECIMATION_MODES * MAX_WEIGHTS_PER_BLOCK];
	temp_buffers.planes2->flt_quantized_decimated_quantized_weights = new float[2 * MAX_WEIGHT_MODES * MAX_WEIGHTS_PER_BLOCK];
	temp_buffers.planes2->u8_quantized_decimated_quantized_weights = new uint8_t[2 * MAX_WEIGHT_MODES * MAX_WEIGHTS_PER_BLOCK];
	temp_buffers.plane1 = temp_buffers.planes2;

	for (z = 0; z < zblocks; z++)
		for (y = 0; y < yblocks; y++)
			for (x = 0; x < xblocks; x++)
			{
				if (ctr == 0)
				{
					int offset = ((z * yblocks + y) * xblocks + x) * 16;
					uint8_t *bp = buffer + offset;
				#ifdef DEBUG_PRINT_DIAGNOSTICS
					if (diagnostics_tile < 0 || diagnostics_tile == pctr)
					{
						print_diagnostics = (diagnostics_tile == pctr) ? 1 : 0;
				#endif
						fetch_imageblock(input_image, &pb, xdim, ydim, zdim, x * xdim, y * ydim, z * zdim, swz_encode);
						symbolic_compressed_block scb;
						compress_symbolic_block(input_image, decode_mode, xdim, ydim, zdim, ewp, &pb, &scb, &temp_buffers);
						if (pack_and_unpack)
						{
							decompress_symbolic_block(decode_mode, xdim, ydim, zdim, x * xdim, y * ydim, z * zdim, &scb, &pb);
							write_imageblock(output_image, &pb, xdim, ydim, zdim, x * xdim, y * ydim, z * zdim, swz_decode);
						}
						else
						{
							physical_compressed_block pcb;
							pcb = symbolic_to_physical(xdim, ydim, zdim, &scb);
							*(physical_compressed_block *) bp = pcb;
						}
				#ifdef DEBUG_PRINT_DIAGNOSTICS
					}
				#endif

					counters[thread_id]++;
					ctr = threadcount - 1;

					pctr++;

					// routine to print the progress counter.
					if (suppress_progress_counter == 0 && (pctr % progress_counter_divider) == 0 && print_tile_errors == 0 && print_statistics == 0)
					{
						int do_print = 1;
						// the current thread has the responsibility for printing the progress counter
						// if every previous thread has completed. Also, if we have ever received the
						// responsibility to print the progress counter, we are going to keep it
						// until the thread is completed.
						if (!owns_progress_counter)
						{
							for (i = thread_id - 1; i >= 0; i--)
							{
								if (threads_completed[i] == 0)
								{
									do_print = 0;
									break;
								}
							}
						}
						if (do_print)
						{
							owns_progress_counter = 1;
							int summa = 0;
							for (i = 0; i < threadcount; i++)
								summa += counters[i];
							printf("\r%d", summa);
							fflush(stdout);
						}
					}
				}
				else
					ctr--;
			}

	delete[] temp_buffers.planes2->decimated_quantized_weights;
	delete[] temp_buffers.planes2->decimated_weights;
	delete[] temp_buffers.planes2->flt_quantized_decimated_quantized_weights;
	delete[] temp_buffers.planes2->u8_quantized_decimated_quantized_weights;
	delete[] temp_buffers.planes2->eix1;
	delete[] temp_buffers.planes2->eix2;
	delete   temp_buffers.planes2->ei1;
	delete   temp_buffers.planes2->ei2;
	delete   temp_buffers.planes2;
	delete[] temp_buffers.tempblocks;
	delete   temp_buffers.temp;
	delete   temp_buffers.ewbo;
	delete   temp_buffers.ewb;

	threads_completed[thread_id] = 1;
	return NULL;
}


void encode_astc_image(const astc_codec_image * input_image,
					   astc_codec_image * output_image,
					   int xdim,
					   int ydim,
					   int zdim,
					   const error_weighting_params * ewp, astc_decode_mode decode_mode, swizzlepattern swz_encode, swizzlepattern swz_decode, uint8_t * buffer, int pack_and_unpack, int threadcount)
{
	int i;
	int *counters = new int[threadcount];
	int *threads_completed = new int[threadcount];

	// before entering into the multi-threaded routine, ensure that the block size descriptors
	// and the partition table descriptors needed actually exist.
	get_block_size_descriptor(xdim, ydim, zdim);
	get_partition_table(xdim, ydim, zdim, 0);

	encode_astc_image_info *ai = new encode_astc_image_info[threadcount];
	for (i = 0; i < threadcount; i++)
	{
		ai[i].xdim = xdim;
		ai[i].ydim = ydim;
		ai[i].zdim = zdim;
		ai[i].buffer = buffer;
		ai[i].ewp = ewp;
		ai[i].counters = counters;
		ai[i].pack_and_unpack = pack_and_unpack;
		ai[i].thread_id = i;
		ai[i].threadcount = threadcount;
		ai[i].decode_mode = decode_mode;
		ai[i].swz_encode = swz_encode;
		ai[i].swz_decode = swz_decode;
		ai[i].threads_completed = threads_completed;
		ai[i].input_image = input_image;
		ai[i].output_image = output_image;
		counters[i] = 0;
		threads_completed[i] = 0;
	}

	if (threadcount == 1)
		encode_astc_image_threadfunc(&ai[0]);
	else
	{
		pthread_t *threads = new pthread_t[threadcount];
		for (i = 0; i < threadcount; i++)
			pthread_create(&(threads[i]), NULL, encode_astc_image_threadfunc, (void *)(&(ai[i])));

		for (i = 0; i < threadcount; i++)
			pthread_join(threads[i], NULL);
		delete[]threads;
	}

	delete[]ai;
	delete[]counters;
	delete[]threads_completed;
}


void store_astc_file(const astc_codec_image * input_image,
					 const char *filename, int xdim, int ydim, int zdim, const error_weighting_params * ewp, astc_decode_mode decode_mode, swizzlepattern swz_encode, int threadcount)
{
	int xsize = input_image->xsize;
	int ysize = input_image->ysize;
	int zsize = input_image->zsize;

	int xblocks = (xsize + xdim - 1) / xdim;
	int yblocks = (ysize + ydim - 1) / ydim;
	int zblocks = (zsize + zdim - 1) / zdim;

	uint8_t *buffer = (uint8_t *) malloc(xblocks * yblocks * zblocks * 16);
	if (!buffer)
	{
		printf("Ran out of memory\n");
		exit(1);
	}

	if (!suppress_progress_counter)
		printf("%d blocks to process ..\n", xblocks * yblocks * zblocks);

	encode_astc_image(input_image, NULL, xdim, ydim, zdim, ewp, decode_mode, swz_encode, swz_encode, buffer, 0, threadcount);

	end_coding_time = get_time();

	astc_header hdr;
	hdr.magic[0] = MAGIC_FILE_CONSTANT & 0xFF;
	hdr.magic[1] = (MAGIC_FILE_CONSTANT >> 8) & 0xFF;
	hdr.magic[2] = (MAGIC_FILE_CONSTANT >> 16) & 0xFF;
	hdr.magic[3] = (MAGIC_FILE_CONSTANT >> 24) & 0xFF;
	hdr.blockdim_x = xdim;
	hdr.blockdim_y = ydim;
	hdr.blockdim_z = zdim;
	hdr.xsize[0] = xsize & 0xFF;
	hdr.xsize[1] = (xsize >> 8) & 0xFF;
	hdr.xsize[2] = (xsize >> 16) & 0xFF;
	hdr.ysize[0] = ysize & 0xFF;
	hdr.ysize[1] = (ysize >> 8) & 0xFF;
	hdr.ysize[2] = (ysize >> 16) & 0xFF;
	hdr.zsize[0] = zsize & 0xFF;
	hdr.zsize[1] = (zsize >> 8) & 0xFF;
	hdr.zsize[2] = (zsize >> 16) & 0xFF;

	FILE *wf = fopen(filename, "wb");
	fwrite(&hdr, 1, sizeof(astc_header), wf);
	fwrite(buffer, 1, xblocks * yblocks * zblocks * 16, wf);
	fclose(wf);
	free(buffer);
}



astc_codec_image *pack_and_unpack_astc_image(const astc_codec_image * input_image,
											 int xdim,
											 int ydim,
											 int zdim,
											 const error_weighting_params * ewp, astc_decode_mode decode_mode, swizzlepattern swz_encode, swizzlepattern swz_decode, int bitness, int threadcount)
{
	int xsize = input_image->xsize;
	int ysize = input_image->ysize;
	int zsize = input_image->zsize;

	astc_codec_image *img = allocate_image(bitness, xsize, ysize, zsize, 0);

	/*
	   allocate_output_image_space( bitness, xsize, ysize, zsize ); */

	int xblocks = (xsize + xdim - 1) / xdim;
	int yblocks = (ysize + ydim - 1) / ydim;
	int zblocks = (zsize + zdim - 1) / zdim;

	if (!suppress_progress_counter)
		printf("%d blocks to process...\n", xblocks * yblocks * zblocks);

	encode_astc_image(input_image, img, xdim, ydim, zdim, ewp, decode_mode, swz_encode, swz_decode, NULL, 1, threadcount);

	if (!suppress_progress_counter)
		printf("\n");

	return img;
}


void find_closest_blockdim_2d(float target_bitrate, int *x, int *y, int consider_illegal)
{
	int blockdims[6] = { 4, 5, 6, 8, 10, 12 };

	float best_error = 1000;
	float aspect_of_best = 1;
	int i, j;

	// Y dimension
	for (i = 0; i < 6; i++)
	{
		// X dimension
		for (j = i; j < 6; j++)
		{
			//              NxN       MxN         8x5               10x5              10x6
			int is_legal = (j==i) || (j==i+1) || (j==3 && i==1) || (j==4 && i==1) || (j==4 && i==2);

			if(consider_illegal || is_legal)
			{
				float bitrate = 128.0f / (blockdims[i] * blockdims[j]);
				float bitrate_error = fabs(bitrate - target_bitrate);
				float aspect = (float)blockdims[j] / blockdims[i];
				if (bitrate_error < best_error || (bitrate_error == best_error && aspect < aspect_of_best))
				{
					*x = blockdims[j];
					*y = blockdims[i];
					best_error = bitrate_error;
					aspect_of_best = aspect;
				}
			}
		}
	}
}



void find_closest_blockdim_3d(float target_bitrate, int *x, int *y, int *z, int consider_illegal)
{
	int blockdims[4] = { 3, 4, 5, 6 };

	float best_error = 1000;
	float aspect_of_best = 1;
	int i, j, k;

	for (i = 0; i < 4; i++)	// Z
		for (j = i; j < 4; j++) // Y
			for (k = j; k < 4; k++) // X
			{
				//              NxNxN              MxNxN                  MxMxN
				int is_legal = ((k==j)&&(j==i)) || ((k==j+1)&&(j==i)) || ((k==j)&&(j==i+1));

				if(consider_illegal || is_legal)
				{
					float bitrate = 128.0f / (blockdims[i] * blockdims[j] * blockdims[k]);
					float bitrate_error = fabs(bitrate - target_bitrate);
					float aspect = (float)blockdims[k] / blockdims[j] + (float)blockdims[j] / blockdims[i] + (float)blockdims[k] / blockdims[i];

					if (bitrate_error < best_error || (bitrate_error == best_error && aspect < aspect_of_best))
					{
						*x = blockdims[k];
						*y = blockdims[j];
						*z = blockdims[i];
						best_error = bitrate_error;
						aspect_of_best = aspect;
					}
				}
			}
}


void compare_two_files(const char *filename1, const char *filename2, int low_fstop, int high_fstop, int psnrmode)
{
	int load_result1;
	int load_result2;
	astc_codec_image *img1 = astc_codec_load_image(filename1, 0, &load_result1);
	if (load_result1 < 0)
	{
		printf("Failed to load file %s.\n", filename1);
		exit(1);
	}
	astc_codec_image *img2 = astc_codec_load_image(filename2, 0, &load_result2);
	if (load_result2 < 0)
	{
		printf("Failed to load file %s.\n", filename2);
		exit(1);
	}

	int file1_components = load_result1 & 0x7;
	int file2_components = load_result2 & 0x7;
	int comparison_components = MAX(file1_components, file2_components);

	int compare_hdr = 0;
	if (load_result1 & 0x80)
		compare_hdr = 1;
	if (load_result2 & 0x80)
		compare_hdr = 1;

	compute_error_metrics(compare_hdr, comparison_components, img1, img2, low_fstop, high_fstop, psnrmode);
}


union if32
{
	float f;
	int32_t s;
	uint32_t u;
};


// The ASTC codec is written with the assumption that a float threaded through
// the "if32" union will in fact be stored and reloaded as a 32-bit IEEE-754 single-precision
// float, stored with round-to-nearest rounding. This is always the case in an
// IEEE-754 compliant system, however not every system is actually IEEE-754 compliant
// in the first place. As such, we run a quick test to check that this is actually the case
// (e.g. gcc on 32-bit x86 will typically fail unless -msse2 -mfpmath=sse2 is specified).

volatile float xprec_testval = 2.51f;
void test_inappropriate_extended_precision(void)
{
	if32 p;
	p.f = xprec_testval + 12582912.0f;
	float q = p.f - 12582912.0f;
	if (q != 3.0f)
	{
		printf("Single-precision test failed; please recompile with proper IEEE-754 support.\n");
		exit(1);
	}
}

// Debug routine to dump the entire image if requested.
void dump_image(astc_codec_image * img)
{
	int x, y, z, xdim, ydim, zdim;

	printf("\n\nDumping image ( %d x %d x %d + %d)...\n\n", img->xsize, img->ysize, img->zsize, img->padding);

	if (img->zsize != 1)
		zdim = img->zsize + 2 * img->padding;
	else
		zdim = img->zsize;

	ydim = img->ysize + 2 * img->padding;
	xdim = img->xsize + 2 * img->padding;

	for (z = 0; z < zdim; z++)
	{
		if (z != 0)
			printf("\n\n");
		for (y = 0; y < ydim; y++)
		{
			if (y != 0)
				printf("\n");
			for (x = 0; x < xdim; x++)
			{
				printf("  0x%08X", *(int unsigned *)&img->imagedata8[z][y][x]);
			}
		}
	}
	printf("\n\n");
}
