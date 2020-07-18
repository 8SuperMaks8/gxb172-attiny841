#ifndef MEDIANFILTER_H_
#define MEDIANFILTER_H_

template<typename DataType, const unsigned char buffer_length = 8u, const unsigned char stopper = 0u>
class MedianFilter {
	private:
		struct rec {
			rec *next;                              /* Pointers forming list linked in sorted order */
			DataType val;                                   /* Values to sort */
		};

		rec buffer[buffer_length]; /* Buffer of nwidth pairs */
		rec *datpoint;               /* Pointer into circular buffer of data */
		rec small;          /* Chain stopper */
		rec big;                /* Pointer to head (largest) of linked list.*/
	
	public:
		MedianFilter(void): buffer{}, datpoint(buffer), small{NULL, stopper}, big{&small, stopper} {}
	
		DataType operator()(DataType data) {
			if ((++datpoint - buffer) >= buffer_length) {
				datpoint = buffer;                               /* Increment and wrap data in pointer.*/
			}

			datpoint->val = data;                           /* Copy in new data */
			rec *successor = datpoint->next;                       /* Save pointer to old value's successor */
			rec *median = &big;                                     /* Median initially to first in chain */
			rec *scanold = NULL;                                    /* Scanold initially null. */
			rec *scan = &big;                                       /* Points to pointer to first (largest) data in chain */

			/* Handle chain-out of first item in chain as special case */
			if (scan->next == datpoint) {
				scan->next = successor;
			}
		
			scanold = scan;
			scan = scan->next;
		
			bool sep = true;
			/* Loop through the chain, normal loop exit via break. */
			for (;;) {
				/* Handle odd-numbered item in chain  */
				if (scan->next == datpoint) {
					scan->next = successor;                      /* Chain out the old data.*/
				}

				if (scan->val < data) {
					datpoint->next = scanold->next;             /* Chain it in here.  */
					scanold->next = datpoint;                    /* Mark it chained in. */
					data = stopper;
				}

				if (sep) {
					/* Step median pointer down chain after doing odd-numbered element */
					median = median->next;                       /* Step median pointer.  */
				}
				/* Handle even-numbered item in chain.  */
				sep = !sep;
			
				if (scan == &small) {
					break;                                      /* Break at end of chain  */
				}
				scanold = scan;                               /* Save this pointer and   */
				scan = scan->next;                           /* step down chain */
			}
			return median->val;
		}
};

template<typename DataType, const unsigned char buffer_length = 8u, const unsigned char stopper = 0u>
class MedianFilteredValue : private MedianFilter<DataType, buffer_length, stopper> {
	private:
		DataType val;

	public:
		MedianFilteredValue(void): val(stopper) {}
		MedianFilteredValue(DataType initial_val): val(initial_val) {}

		DataType operator()(DataType data) {
			return val = MedianFilter<DataType, buffer_length, stopper>::operator()(data);
		}

		DataType operator()(void) {
			return val;
		}
};

#endif /* MEDIANFILTER_H_ */