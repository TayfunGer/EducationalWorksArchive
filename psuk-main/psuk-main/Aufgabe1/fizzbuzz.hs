import System.Directory.Internal.Prelude (Integral)
fizzBuzz :: (Integral a, Show a) => a -> [Char]
fizzBuzz x | mod x 15 == 0 = "FizzBuzz"
           | mod x 5 == 0 = "Buzz"
           | mod x 3 == 0 = "Fizz"
           | otherwise = show x

