{-# OPTIONS_GHC -Wno-incomplete-patterns #-}
import GHC.Real (infinity)
import Language.Haskell.TH (anyclassStrategy)
import Data.List ( sort, sortBy )
import Distribution.Simple.Utils (xargs)
import qualified Data.IntMap as Map

-- Aufgabe 1

data IntervalMap k v = IntervalMap v [(k,v)]
   deriving (Show, Eq, Ord)

singleton :: v -> IntervalMap k v
singleton v = IntervalMap v []

(!) :: Ord k => IntervalMap k v -> k -> v
(!) (IntervalMap d []) _ = d
(!) (IntervalMap d xs) key | fst (head xs) > key || fst (last xs) < key = d
                           | fst(xs !! 1) < key = (!) (IntervalMap d (drop 1 xs)) key
                           | fst(xs !! 1) == key = snd (xs !! 1)
                           | otherwise = snd (head xs)

-- insertList :: Ord k => k -> k -> v -> [(k,v)] -> [(k,v)]
-- insertList _ _ _ [] = []
-- insertList begin end value xs | fst (head xs) < begin = head xs : insertList begin end value (tail xs)
--                               | fst (head xs) > begin && null (tail xs) = (begin, value) : xs

getEndVal :: Ord k => k -> v -> [(k,v)] -> v
getEndVal end d xs | fst (last xs) < end = d
                   | fst (last xs) >= end && fst (reverse xs !! 1) <= end = snd (reverse xs !! 1)
                   | otherwise = getEndVal end d (reverse (drop 1 (reverse xs)))

sorter :: Ord k => [(k,v)] -> [(k,v)]
sorter [] = []
sorter xs = sortBy (\(a, _) (b, _) -> compare a b) xs

sortInsert :: Ord k => k -> k -> v -> v -> [(k,v)] -> [(k,v)]
sortInsert begin end value d xs = sorter ([(begin, value), (end, getEndVal end d xs)] ++ eraseOverlap begin end xs)

eraseOverlap :: Ord k => k -> k -> [(k,v)] -> [(k,v)]
eraseOverlap _ _ [] = []
eraseOverlap begin end xs | fst (head xs) >= begin && fst (head xs) <= end = eraseOverlap begin end (tail xs)
                          | otherwise = head xs : eraseOverlap begin end (tail xs)

-- insertBegin :: Ord k => k -> k -> v -> [(k,v)] -> [(k,v)]
-- insertBegin _ _ _ [] = []
-- insertBegin begin end value xs | fst (head xs) < begin = eraseOverlap begin end [head xs] ++ insertBegin begin end value (tail xs)
--                          | fst (head xs) >= begin = (begin, value) : eraseOverlap begin end xs

-- insertEnd :: Ord k => k -> v -> v -> [(k,v)] -> [(k,v)]
-- insertEnd end value d xs | fst (last xs) > end = insertEnd end value d (reverse (drop 1 (reverse xs))) ++ [last xs]
--                          | fst (last xs) <= end = xs ++ [(end, x)]
--    where x | null (tail xs) = d
--            | otherwise = snd (reverse xs !! 1)

insert :: Ord k => k -> k -> v -> IntervalMap k v -> IntervalMap k v
insert begin end value (IntervalMap d []) = IntervalMap d [(begin, value), (end,d)]
insert begin end value (IntervalMap d xs) = IntervalMap d (sortInsert begin end value d xs)


a :: IntervalMap Int Char
a = singleton 'a' :: IntervalMap Int Char
b :: IntervalMap Int Char
b = insert 10 20 'b' a
c :: IntervalMap Int Char
c = insert 9 21 'c' b
d :: IntervalMap Int Char
d = insert 5 15 'd' c
e :: IntervalMap Int Char
e = insert 14 22 'e' d
f :: IntervalMap Int Char
f = insert 10 19 'f' e

-- Aufgabe 2

-- myMap :: (v -> w) -> [(k,v)] -> [(k,w)]
-- myMap f xs = map (\(_, second) -> f second) xs

myFunc1 :: (a -> b) -> [(k,a)] -> [(k,b)]
myFunc1 _ [] = []
myFunc1 f xs = fmap f (head xs) : myFunc1 f (tail xs)

myFunc2 :: a -> [(k,b)] -> [(k,a)]
myFunc2 _ [] = []
myFunc2 a xs = (fst (head xs), a) : myFunc2 a (tail xs)

instance Functor (IntervalMap k) where
   fmap f (IntervalMap d xs) = IntervalMap (f d) (myFunc1 f xs)
   (<$) b (IntervalMap d xs) = IntervalMap b (myFunc2 b xs)

g :: IntervalMap Int Int
g = fmap fromEnum f
h :: IntervalMap Int [Char]
h = "Hello" <$ g

-- Aufgabe 3

getChanges :: Ord k => [(k, a -> b)] -> [(k, a)] -> [k]
getChanges [] [] = []
getChanges [] bx = fst (head bx) : getChanges [] (drop 1 bx)
getChanges ax [] = fst (head ax) : getChanges [] (drop 1 ax)
getChanges ax bx = unique(sort([fst (head ax), fst (head bx)] ++ getChanges (drop 1 ax) (drop 1 bx)))

unique :: Ord k => [k] -> [k]
unique xs = [x | (x,y) <- zip xs [0..], x `notElem` take y xs]

makeIntervalMap :: Ord k => v -> [k] -> IntervalMap k (t -> v) -> IntervalMap k t -> IntervalMap k v
makeIntervalMap d [] _ _ = IntervalMap d []
makeIntervalMap d ks (IntervalMap ad ax) (IntervalMap bd bx) = IntervalMap d [(k, (IntervalMap ad ax ! k) (IntervalMap bd bx ! k)) | k <- ks]

instance Ord k => Applicative (IntervalMap k) where
   pure x = IntervalMap x mempty
   (<*>) (IntervalMap ad ax) (IntervalMap bd bx) = makeIntervalMap (ad bd) (getChanges ax bx) (IntervalMap ad ax) (IntervalMap bd bx)

i :: IntervalMap Int Int
i = insert 5 10 110 $ insert 10 15 90 $ singleton 100 :: IntervalMap Int Int
j :: IntervalMap Int (Int -> Int -> Int)
j = insert 5 10 (-) $ insert 10 15 (*) $ singleton (+) :: IntervalMap Int (Int -> Int -> Int)
k :: IntervalMap Int Int
k = insert 3 18 2 $ singleton 10 :: IntervalMap Int Int
