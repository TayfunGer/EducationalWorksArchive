{-# OPTIONS_GHC -Wno-incomplete-patterns #-}
isPrime :: Int -> Bool 
isPrime n | null[x | x <- [2..n-1], mod n x == 0] = True 
          | otherwise = False

primes :: [Int]
primes = [x | x <- [2..], isPrime x]

primeFactors :: Integral a => a -> [a]
primeFactors n | n == 0 = []
               | n < 0 = primeFactors (-n)
               | n > 0 = if 1 == n then []
                         else let fac = mfac n 2 in fac : primeFactors (div n fac)
    where mfac m x | mod m x == 0 = x
                   | x * x > m = m
                   | otherwise = mfac m (if odd x then x + 2 else x + 1)
