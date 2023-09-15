using System.Buffers;
using System;
using System.Collections;
using System.Collections.Generic;
using GameFramework;
using System.Collections.Concurrent;

/// <summary>
/// 数组池, 可以避免频繁new数组
/// </summary>
/// <typeparam name="T"></typeparam>
public class CacheArray<T> : IList<T>, IReference
{
    public T this[int index]
    {
        get
        {
            lock (SyncRoot)
            {
                return _items[index];
            }
        }
        set
        {
            lock (SyncRoot)
            {
                _items[index] = value;
            }
        }
    }

    private T[] _items;
    private int _size;

    public bool IsFixedSize => false;

    public bool IsReadOnly => false;

    public int Count => _size;

    public bool IsSynchronized => false;

    public object SyncRoot => _items;

    public CacheArray()
    {

    }
    public static CacheArray<T> Acquire(int currentSize, int cacheSize)
    {
        var arr = ReferencePool.Acquire<CacheArray<T>>();
        arr._items = ArrayPool<T>.Shared.Rent(cacheSize);
        arr._size = currentSize;
        return arr;
    }
    /// <summary>
    /// 不用时释放并归还数组到数组池
    /// </summary>
    public void Release()
    {
        lock (SyncRoot)
        {
            ReferencePool.Release(this);
        }
    }
    public void Resize(int capacity)
    {
        lock (SyncRoot)
        {
            if (_size > capacity)
            {
                Array.Clear(_items, capacity - 1, _size - capacity);
                _size = capacity;
                return;
            }

            if (capacity > _size)
            {
                int newArrLength = capacity;
                var newArr = ArrayPool<T>.Shared.Rent(newArrLength);
                Array.Copy(_items, newArr, _size);
                ArrayPool<T>.Shared.Return(_items, true);
                _items = newArr;
                _size = newArrLength;
            }
        }
    }
    public void Add(T item)
    {
        lock (SyncRoot)
        {
            if (_items.Length > _size)
            {
                _items[_size++] = item;
                return;
            }

            int newArrLength = _size + 1;
            var newArr = ArrayPool<T>.Shared.Rent(newArrLength);
            Array.Copy(_items, newArr, _size);
            ArrayPool<T>.Shared.Return(_items, true);

            _items = newArr;
            _size = newArrLength;
        }

    }

    public void Clear()
    {
        lock (SyncRoot)
        {
            if (_size > 0 && _items != null)
            {
                ArrayPool<T>.Shared.Return(_items, true);
                _size = 0;
            }
        }
    }

    public bool Contains(T item)
    {
        lock (SyncRoot)
        {
            if (item == null)
            {
                for (int i = 0; i < _size; i++)
                {
                    if (_items[i] == null)
                    {
                        return true;
                    }
                }

                return false;
            }

            EqualityComparer<T> @default = EqualityComparer<T>.Default;
            for (int j = 0; j < _size; j++)
            {
                if (@default.Equals(_items[j], item))
                {
                    return true;
                }
            }

            return false;
        }

    }

    public void CopyTo(T[] array, int arrayIndex)
    {
        lock (SyncRoot)
        {
            Array.Copy(_items, 0, array, arrayIndex, _size);
        }
    }

    public int IndexOf(T item)
    {
        lock (SyncRoot)
        {
            return System.Array.IndexOf(_items, item);
        }
    }

    public void Insert(int index, T item)
    {
        lock (SyncRoot)
        {
            if (_items.Length > _size)
            {
                for (int i = _size - 1; i >= index; i--)
                {
                    _items[i + 1] = _items[i];
                }
                _items[index] = item;
                return;
            }

            int newArrLength = _size + 1;
            var newArr = ArrayPool<T>.Shared.Rent(newArrLength);
            for (int i = _size - 1; i >= index; i--)
            {
                newArr[i + 1] = _items[i];
            }
            newArr[index] = item;
            ArrayPool<T>.Shared.Return(_items, true);
            _items = newArr;
            _size = newArrLength;
        }

    }

    public bool Remove(T item)
    {
        lock (SyncRoot)
        {
            int idx = Array.IndexOf(_items, item);
            if (idx >= 0 && idx < _size)
            {
                RemoveAt(idx);
                return true;
            }
            return false;
        }

    }

    public void RemoveAt(int index)
    {
        lock (SyncRoot)
        {
            if (index == _size - 1)
            {
                _items[index] = default;
                _size -= 1;
                return;
            }
            for (int i = index + 1; i < _size; i++)
            {
                _items[i - 1] = _items[i];
            }
            _size -= 1;
        }
    }

    IEnumerator IEnumerable.GetEnumerator()
    {
        throw new NotSupportedException("Not Supported Enumerator");
    }

    public IEnumerator<T> GetEnumerator()
    {
        throw new NotSupportedException("Not Supported Enumerator");
    }
}