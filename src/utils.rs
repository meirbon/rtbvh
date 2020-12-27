use num::*;
use std::ops::{Index, IndexMut};
use std::sync::Arc;
use std::{
    ops::Range,
    sync::atomic::{AtomicUsize, Ordering},
};

#[cfg(not(feature = "wasm_support"))]
use rayon::prelude::*;

pub fn round_up_log2(bits: u32, mut offset: u32) -> u32 {
    if bits == 0 {
        offset
    } else {
        while (1 << offset) < bits {
            offset = offset + 1;
        }

        offset
    }
}

#[allow(dead_code)]
#[cfg(feature = "wasm_support")]
pub fn shuffle_prims<T: Sized + Copy + Send + Sync>(primitives: &[T], indices: &[u32]) -> Vec<T> {
    (0..indices.len())
        .into_iter()
        .map(|i| primitives[indices[i] as usize].clone())
        .collect()
}

#[allow(dead_code)]
#[cfg(not(feature = "wasm_support"))]
pub fn shuffle_prims<T: Sized + Copy + Send + Sync>(primitives: &[T], indices: &[u32]) -> Vec<T> {
    (0..indices.len())
        .into_par_iter()
        .map(|i| primitives[indices[i] as usize].clone())
        .collect()
}

pub fn prefix_sum<T: Num + Sized + Copy>(first: &[T], count: usize, output: &mut [T]) -> T {
    debug_assert!(first.len() >= count);
    debug_assert!(output.len() >= count);

    if count.is_zero() {
        return first[0];
    }

    let mut sum: T = T::zero();
    for i in 0..count {
        sum = sum.add(first[i]);
        output[i] = sum;
    }

    sum
}

pub unsafe fn move_backward<T: Sized + Clone>(
    first: *mut T,
    mut last: *mut T,
    mut d_last: *mut T,
) -> *mut T {
    while first != last {
        d_last = d_last.sub(1);
        last = last.sub(1);

        std::ptr::write(d_last, (*last).clone());
    }

    d_last
}

/// Partitions range of slice according to given check.
/// Returns how many elements went left.
pub fn partition<T: Sized + Clone, B>(slice: &mut [T], range: Range<usize>, check: B) -> usize
where
    B: Fn(&T) -> bool,
{
    debug_assert!(
        slice.len() >= (range.end - range.start),
        "Slice was smaller ({}) than range ({})",
        slice.len(),
        range.end - range.start
    );

    let mut count: usize = 0;
    for i in range {
        if check(&slice[i]) {
            slice.swap(i, count);
            count += 1;
        }
    }

    count
}

#[derive(Debug, Clone)]
pub struct UnsafeSliceWrapper<'a, T: Sized> {
    ptr: *mut T,
    slice: &'a [T],
}

impl<'a, T> Index<usize> for UnsafeSliceWrapper<'a, T> {
    type Output = T;

    fn index(&self, index: usize) -> &Self::Output {
        &self.slice[index]
    }
}

impl<'a, T> IndexMut<usize> for UnsafeSliceWrapper<'a, T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        self.get_mut(index).unwrap()
    }
}

#[allow(dead_code)]
impl<'a, T: Sized> UnsafeSliceWrapper<'a, T> {
    pub fn new(array: &'a mut [T]) -> Self {
        Self {
            ptr: array.as_mut_ptr(),
            slice: array,
        }
    }

    pub fn len(&self) -> usize {
        self.slice.len()
    }

    pub fn get(&self, idx: usize) -> Option<&'a T> {
        debug_assert!(idx < self.slice.len());
        unsafe { self.ptr.add(idx).as_ref() }
    }

    pub fn get_mut(&self, idx: usize) -> Option<&'a mut T> {
        debug_assert!(idx < self.slice.len());
        unsafe { self.ptr.add(idx).as_mut() }
    }

    pub fn set(&self, idx: usize, val: T) {
        debug_assert!(idx < self.slice.len());
        unsafe {
            std::ptr::write(self.ptr.add(idx), val);
        }
    }

    pub fn as_slice(&self) -> &[T] {
        self.slice
    }

    pub fn as_mut(&self) -> &mut [T] {
        unsafe { std::slice::from_raw_parts_mut(self.ptr, self.len()) }
    }

    pub fn as_ptr(&self) -> *const T {
        self.ptr as *const T
    }

    pub fn as_mut_ptr(&self) -> *mut T {
        self.ptr
    }

    pub fn swap(&self, a: usize, b: usize) {
        debug_assert!(a < self.slice.len());
        debug_assert!(b < self.slice.len());
        self.as_mut().swap(a, b);
    }

    pub fn range(&self, start: usize, end: usize) -> &mut [T] {
        debug_assert!(start < end, "start: {}, end: {}", start, end);
        debug_assert!(
            end <= self.len(),
            "start: {}, end: {}, len: {}",
            start,
            end,
            self.len()
        );
        unsafe { std::slice::from_raw_parts_mut(self.ptr.add(start), end - start) }
    }
}

unsafe impl<'a, T> Send for UnsafeSliceWrapper<'a, T> {}

unsafe impl<'a, T> Sync for UnsafeSliceWrapper<'a, T> {}

pub struct TaskSpawner {
    pub threads_in_flight: Arc<AtomicUsize>,
    config: TaskConfig,
}

#[derive(Debug, Copy, Clone)]
struct TaskConfig {
    pub work_size_threshold: usize,
    pub max_depth: usize,
    pub max_leaf_size: usize,
}

pub trait Task: Sized + Send + Sync {
    fn run(self) -> Option<(Self, Self)>;
    fn work_size(&self) -> usize;
    fn depth(&self) -> usize;
}

#[allow(dead_code)]
impl TaskSpawner {
    pub fn new() -> Self {
        Self {
            config: TaskConfig {
                work_size_threshold: 1024,
                max_depth: 64,
                max_leaf_size: 16,
            },
            threads_in_flight: Arc::new(AtomicUsize::new(0)),
        }
    }

    pub fn with_work_size_threshold(mut self, threshold: usize) -> Self {
        self.config.work_size_threshold = threshold;
        self
    }

    pub fn with_max_depth(mut self, depth: usize) -> Self {
        self.config.max_depth = depth;
        self
    }

    pub fn with_max_leaf_size(mut self, max_leaf_size: usize) -> Self {
        self.config.max_leaf_size = max_leaf_size;
        self
    }

    pub fn run<T: Task>(&self, first_task: T) {
        let thread_count = self.threads_in_flight.clone();
        crossbeam::scope(move |s| {
            Self::run_task(first_task, self.config, thread_count, s);
        })
        .unwrap();
    }

    fn run_task<'a, T: Task + Sized + 'a>(
        task: T,
        config: TaskConfig,
        thread_count: Arc<AtomicUsize>,
        scope: &crossbeam::thread::Scope<'a>,
    ) {
        thread_count.fetch_add(1, Ordering::SeqCst);
        let mut sub_tasks = Vec::new();

        let mut stack: Vec<T> = Vec::new();
        stack.push(task);

        while !stack.is_empty() {
            let work_item = stack.pop().unwrap();
            debug_assert!(work_item.depth() <= config.max_depth);

            if let Some((mut task_a, mut task_b)) = work_item.run() {
                if task_a.work_size() < task_b.work_size() {
                    // Push more work to new thread
                    std::mem::swap(&mut task_a, &mut task_b);
                }

                stack.push(task_b);

                // Remove mutability
                let task_a = task_a;

                // If threshold is not met, push to stack instead of spawning new thread
                if task_a.work_size() <= config.work_size_threshold {
                    stack.push(task_a);
                    continue;
                }

                // Spawn new thread
                let count = thread_count.clone();
                sub_tasks.push(scope.spawn(move |s| {
                    Self::run_task(task_a, config, count, s);
                }));
            }
        }

        // Join thread handles
        while !sub_tasks.is_empty() {
            let r = sub_tasks.pop().unwrap();
            r.join().unwrap();
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::utils::*;

    #[test]
    fn prefix_sum_u32_works() {
        type TestType = u32;

        let input: [TestType; 6] = [1, 2, 3, 4, 5, 6];
        let output: [TestType; 6] = [1, 3, 6, 10, 15, 21];

        let mut storage: Vec<TestType> = vec![0; 6];

        prefix_sum(&input, 6, storage.as_mut_slice());
        for i in 0..6 {
            assert_eq!(output[i], storage[i]);
        }
    }

    #[test]
    fn prefix_sum_usize_works() {
        type TestType = usize;

        let input: [TestType; 6] = [1, 2, 3, 4, 5, 6];
        let output: [TestType; 6] = [1, 3, 6, 10, 15, 21];

        let mut storage: Vec<TestType> = vec![0; 6];

        prefix_sum(&input, 6, storage.as_mut_slice());
        for i in 0..6 {
            assert_eq!(output[i], storage[i]);
        }
    }

    #[test]
    fn prefix_sum_i32_works() {
        type TestType = i32;

        let input: [TestType; 6] = [1, 2, 3, 4, 5, 6];
        let output: [TestType; 6] = [1, 3, 6, 10, 15, 21];

        let mut storage: Vec<TestType> = vec![0; 6];

        prefix_sum(&input, 6, storage.as_mut_slice());
        for i in 0..6 {
            assert_eq!(output[i], storage[i]);
        }
    }

    #[test]
    fn prefix_sum_zero() {
        let input: [u32; 6] = [1, 2, 3, 4, 5, 6];
        let mut storage: Vec<u32> = vec![0; 6];
        assert_eq!(input[0], prefix_sum(&input, 0, storage.as_mut()))
    }

    #[test]
    fn test_move_backwards() {
        let mut src: [u32; 3] = [0, 1, 2];
        let mut dest: [u32; 3] = [0; 3];

        for i in 0..3 {
            assert_eq!(src[i], i as u32);
            assert_eq!(dest[i], 0);
        }

        unsafe {
            move_backward(
                src.as_mut_ptr(),
                src.as_mut_ptr().add(src.len()),
                dest.as_mut_ptr().add(dest.len()),
            );
        }

        for i in 0..3 {
            assert_eq!(src[i], dest[i]);
        }
    }
}
