{-# OPTIONS_GHC -Wno-incomplete-patterns #-}
myhead :: [a] -> a
myhead (x:xs) = x

mytail :: [a] -> [a]
mytail [] = []
mytail (x:xs) = xs

myreverse :: [a] -> [a]
myreverse [] = []
myreverse [x] = [x]
myreverse (x:xs) = myreverse xs ++ [x]

mylast :: [a] -> a
mylast x = myhead (myreverse x)
myinit :: [a] -> [a]
myinit x = myreverse (mytail (myreverse x))
