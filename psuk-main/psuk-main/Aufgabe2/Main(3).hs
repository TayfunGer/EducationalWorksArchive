{-# OPTIONS_GHC -Wno-incomplete-patterns #-}

text = "Vqkkh dvgeg mckeb mmduk ymWil sigzb mlbeS tsnny sjeiv jmg,t amkdh qbdox mvxnp hzxig dUxnz daxhx m,pal fmmag vmkdx munsl .-Tlt mBnrb mo"

addSpaceAt :: Int -> [Char] -> [Char]
addSpaceAt _ [] = []
addSpaceAt n xs | length xs > 5 = take n xs ++ " " ++ addSpaceAt n (drop n xs)
                | otherwise = xs

crypt :: Int -> [Char] -> [Char]
crypt _ [] = []
crypt key (x:xs) | num > 64 && num < 91 = toEnum (mod (num - 65 + key) 26 + 65) : crypt key xs
                         | num > 96 && num < 123 = toEnum (mod (num - 97 + key) 26 + 97) : crypt key xs
                         | otherwise = x : crypt key xs
                    where num = fromEnum x

decrypt :: Int -> [Char] -> [Char]
decrypt _ [] = []
decrypt key (x:xs) | num > 64 && num < 91 = toEnum (mod (num - 65 - key) 26 + 65) : decrypt key xs
                         | num > 96 && num < 123 = toEnum (mod (num - 97 - key) 26 + 97) : decrypt key xs
                         | otherwise = x : decrypt key xs
                    where num = fromEnum x

caeser :: (Int -> [Char] -> [Char]) -> Int -> [Char] -> [Char]
caeser f key xs = addSpaceAt 5 (f key (filter (\x -> x /= ' ') xs))


helperV :: (Int -> [Char] -> [Char]) -> [Char] -> [Char] -> [Char]
helperV _ _ [] = []
helperV f (key:keyList) (x:xs) = f (fromEnum key - fromEnum 'a') [x] ++ helperV f keyList xs

viginere :: (Int -> [Char] -> [Char]) -> [Char] -> [Char] -> [Char]
viginere _ _ [] = []
viginere f key text = addSpaceAt 5 (helperV f (take (length text) (cycle key)) (filter (\x -> x /= ' ') text))
