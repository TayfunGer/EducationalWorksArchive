import Data.List
import Data.Time.Clock.POSIX
import System.IO
import System.Process

main :: IO ()
main = do
  system "clear"
  putStrLn "_|_|_|_|_|  _|            _|_|_|_|_|                    _|_|_|_|_|                  \n    _|            _|_|_|      _|      _|_|_|    _|_|_|      _|      _|_|      _|_|  \n    _|      _|  _|            _|    _|    _|  _|            _|    _|    _|  _|_|_|_|\n    _|      _|  _|            _|    _|    _|  _|            _|    _|    _|  _|      \n    _|      _|    _|_|_|      _|      _|_|_|    _|_|_|      _|      _|_|      _|_|_|\n"

  firstChoice <- prompt "Welcome to our Haskell TicTacToe!\nChoose mode: (1) KI Gerland, (2) KI Baylan, (3) PVP\n\n\n-> "
  startGame (getMode firstChoice)

------------------------------------------------
--- Start: Data
------------------------------------------------
data Difficulty = Easy | Impossible
  deriving (Eq)

data Mode = PvKI Difficulty | PvP
  deriving (Eq)

data Move = X | O
  deriving (Eq, Show)

data Cell = OccupiedBy Move | Free
  deriving (Eq)

instance Show Cell where
  show (OccupiedBy X) = "X"
  show (OccupiedBy O) = "O"
  show Free = " "

------------------------------------------------
--- End: Data
------------------------------------------------

------------------------------------------------
--- Start: Functions
------------------------------------------------
prompt :: String -> IO String
prompt text = do
  putStr text
  hFlush stdout
  getLine

getMode :: String -> Maybe Mode
getMode "1" = Just (PvKI Easy)
getMode "2" = Just (PvKI Impossible)
getMode "3" = Just PvP
getMode _ = Nothing

startGame :: Maybe Mode -> IO ()
startGame (Just mode) = playRound mode "" X (replicate 9 Free) (0, 0)
startGame _ = do
  system "clear"
  newChoice <- prompt "Welcome to our Haskell TicTacToe!\nChoose mode: (1) KI Gerland, (2) KI Baylan, (3) PVP\n\nWrong input! Try again..\n-> "
  startGame (getMode newChoice)

getFieldIdx :: String -> Maybe Int
getFieldIdx "A1" = Just 0
getFieldIdx "A2" = Just 1
getFieldIdx "A3" = Just 2
getFieldIdx "B1" = Just 3
getFieldIdx "B2" = Just 4
getFieldIdx "B3" = Just 5
getFieldIdx "C1" = Just 6
getFieldIdx "C2" = Just 7
getFieldIdx "C3" = Just 8
getFieldIdx _ = Nothing

verifyIsFree :: Int -> [Cell] -> Bool
verifyIsFree idx field
  | field !! idx == Free = True
  | otherwise = False

isCorrectInput :: Maybe Int -> [Cell] -> Bool
isCorrectInput Nothing field = False
isCorrectInput (Just idx) field = verifyIsFree idx field

addMove :: Move -> Maybe Int -> [Cell] -> [Cell]
addMove move (Just idx) field = take idx field ++ [OccupiedBy move] ++ (drop (idx + 1) field)

incrementScore :: Move -> (Int, Int) -> (Int, Int)
incrementScore X score = (fst score + 1, snd score)
incrementScore O score = (fst score, snd score + 1)

otherMove :: Move -> Move
otherMove X = O
otherMove O = X

isSameMove :: [Int] -> Move -> [Cell] -> Bool
isSameMove [] _ _ = True
isSameMove (x : xs) move field = field !! x == (OccupiedBy move) && isSameMove xs move field

isThereAWinner :: Move -> [Cell] -> Bool
isThereAWinner move field =
  isSameMove [0, 1, 2] move field
    || isSameMove [3, 4, 5] move field
    || isSameMove [6, 7, 8] move field
    || isSameMove [0, 3, 6] move field
    || isSameMove [1, 4, 7] move field
    || isSameMove [2, 5, 8] move field
    || isSameMove [0, 4, 8] move field
    || isSameMove [2, 4, 6] move field

hasPlayerXWon :: [Cell] -> Bool
hasPlayerXWon field = isThereAWinner X field

hasPlayerOWon :: [Cell] -> Bool
hasPlayerOWon field = isThereAWinner O field

nothingIsFree :: [Cell] -> Bool
nothingIsFree [] = True
nothingIsFree (x : xs)
  | x /= Free = True && nothingIsFree xs
  | otherwise = False

------------------------------------------------
--- End: Functions
------------------------------------------------

------------------------------------------------
--- Start: Interface
------------------------------------------------
generateSpaces :: Int -> String
generateSpaces count = take count $ cycle " "

renderRow :: [Cell] -> String
renderRow [] = generateSpaces 14 ++ "     |     |     \n"
renderRow row = generateSpaces 16 ++ (intercalate "  |  " (fmap show row)) ++ "\n"

renderPartingLine :: String
renderPartingLine = generateSpaces 14 ++ "-----|-----|-----\n"

drawField :: [Cell] -> IO ()
drawField field =
  putStr
    ( renderRow []
        ++ renderRow firstRow
        ++ renderPartingLine
        ++ renderRow secondRow
        ++ renderPartingLine
        ++ renderRow thirdRow
        ++ renderRow []
    )
  where
    firstRow = take 3 field
    secondRow = take 3 . drop 3 $ field
    thirdRow = drop 6 field

drawScore :: (Int, Int) -> IO ()
drawScore score =
  putStrLn
    ( "+---------------------+---------------------+\n"
        ++ "  Score: Player 1 = "
        ++ scorePlayerOne
        ++ ", Player 2 = "
        ++ scorePlayerTwo
        ++ ".\n"
        ++ "+-------------------------------------------+\n"
    )
  where
    scorePlayerOne = show $ fst score
    scorePlayerTwo = show $ snd score

drawMainPage :: [Cell] -> (Int, Int) -> IO ()
drawMainPage field score =
  system "clear"
    >> putStrLn
      ( "+-------------------------------------------+\n"
          ++ "| Haskell Tic Tac Toe by Gerland and Baylan |\n"
          ++ "+---------------------+---------------------+"
      )
    >> drawField field
    >> drawScore score

------------------------------------------------
--- End: Interface
------------------------------------------------

------------------------------------------------
--- Start: KI
------------------------------------------------
-- Mit Zeitabfrage wäre das Spiel nicht mehr frei von Nebenwirkungen

--myMod :: Int -> Int -> Int
--myMod second first = mod first second

--randomFreeIdx :: [Cell] -> IO [Cell]
--randomFreeIdx field = fmap fromIntegral (fmap (myMod $ length freeCells) (fmap round --getPOSIXTime))
--            where freeCells = getFreeCellsIdx field 0

-- Easy
getFreeCellsIdx :: [Cell] -> Int -> [Int]
getFreeCellsIdx [] _ = []
getFreeCellsIdx (x : xs) counter
  | x == Free = [counter] ++ getFreeCellsIdx xs (counter + 1)
  | otherwise = getFreeCellsIdx xs (counter + 1)

pickOneFreeCell :: [Cell] -> Int -> Maybe Int
pickOneFreeCell field counter = Just (freeCells !! (mod counter (length freeCells)))
  where
    freeCells = getFreeCellsIdx field 0

-- Ultimate
winIdx :: Move -> [Cell] -> Maybe Int
winIdx move field
  | ( isSameMove [1, 2] move field
        || isSameMove [4, 8] move field
        || isSameMove [3, 6] move field
    )
      && verifyIsFree 0 field =
    Just 0
  | ( isSameMove [0, 2] move field
        || isSameMove [4, 7] move field
    )
      && verifyIsFree 1 field =
    Just 1
  | ( isSameMove [0, 1] move field
        || isSameMove [4, 6] move field
        || isSameMove [5, 8] move field
    )
      && verifyIsFree 2 field =
    Just 2
  | ( isSameMove [0, 6] move field
        || isSameMove [4, 5] move field
    )
      && verifyIsFree 3 field =
    Just 3
  | ( isSameMove [0, 8] move field
        || isSameMove [1, 7] move field
        || isSameMove [2, 6] move field
        || isSameMove [3, 5] move field
    )
      && verifyIsFree 4 field =
    Just 4
  | ( isSameMove [2, 8] move field
        || isSameMove [3, 4] move field
    )
      && verifyIsFree 5 field =
    Just 5
  | ( isSameMove [0, 3] move field
        || isSameMove [2, 4] move field
        || isSameMove [7, 8] move field
    )
      && verifyIsFree 6 field =
    Just 6
  | ( isSameMove [1, 4] move field
        || isSameMove [6, 8] move field
    )
      && verifyIsFree 7 field =
    Just 7
  | ( isSameMove [0, 4] move field
        || isSameMove [2, 5] move field
        || isSameMove [6, 7] move field
    )
      && verifyIsFree 8 field =
    Just 8
  | otherwise = Nothing

defendIdx :: Move -> [Cell] -> Maybe Int
defendIdx move field = winIdx (otherMove move) field

otherwiseIdx :: Move -> [Cell] -> Maybe Int
otherwiseIdx move field
  | verifyIsFree 4 field = Just 4
  | ( isSameMove [0, 8] (otherMove move) field
        || isSameMove [2, 6] (otherMove move) field
    )
      && verifyIsFree 1 field =
    Just 1
  | ( isSameMove [0, 8] (otherMove move) field
        || isSameMove [2, 6] (otherMove move) field
    )
      && verifyIsFree 3 field =
    Just 3
  | ( isSameMove [0, 8] (otherMove move) field
        || isSameMove [2, 6] (otherMove move) field
    )
      && verifyIsFree 5 field =
    Just 5
  | ( isSameMove [0, 8] (otherMove move) field
        || isSameMove [2, 6] (otherMove move) field
    )
      && verifyIsFree 7 field =
    Just 7
  | isSameMove [3, 7] (otherMove move) field && verifyIsFree 6 field = Just 6
  | isSameMove [1, 3] (otherMove move) field && verifyIsFree 0 field = Just 0
  | isSameMove [1, 5] (otherMove move) field && verifyIsFree 2 field = Just 2
  | isSameMove [5, 7] (otherMove move) field && verifyIsFree 8 field = Just 8
  | isSameMove [5, 6] (otherMove move) field && verifyIsFree 8 field = Just 8
  | isSameMove [3, 8] (otherMove move) field && verifyIsFree 6 field = Just 6
  | isSameMove [1, 8] (otherMove move) field && verifyIsFree 2 field = Just 2
  | isSameMove [7, 2] (otherMove move) field && verifyIsFree 8 field = Just 8
  | isSameMove [5, 0] (otherMove move) field && verifyIsFree 2 field = Just 2
  | isSameMove [3, 2] (otherMove move) field && verifyIsFree 0 field = Just 0
  | isSameMove [1, 6] (otherMove move) field && verifyIsFree 0 field = Just 0
  | isSameMove [7, 0] (otherMove move) field && verifyIsFree 6 field = Just 6
  | verifyIsFree 0 field = Just 0
  | verifyIsFree 2 field = Just 2
  | verifyIsFree 6 field = Just 6
  | verifyIsFree 8 field = Just 8
  | otherwise = Nothing

nextKiStep :: Maybe Int -> Maybe Int -> Maybe Int -> Maybe Int
nextKiStep (Just idx) _ _ = (Just idx)
nextKiStep _ (Just idx) _ = (Just idx)
nextKiStep _ _ (Just idx) = (Just idx)
nextKiStep _ _ _ = Nothing

ultraKi :: Move -> [Cell] -> Maybe Int
ultraKi kiMove field = nextKiStep (winIdx kiMove field) (defendIdx kiMove field) (otherwiseIdx kiMove field)

resetWhenKiWon :: Move -> [Cell] -> [Cell]
resetWhenKiWon kiMove field
  | isThereAWinner kiMove field = replicate 9 Free
  | otherwise = field

--test = [OccupiedBy X, OccupiedBy O, Free, Free, Free, Free, OccupiedBy X, OccupiedBy O, Free]

------------------------------------------------
--- End: KI
------------------------------------------------

------------------------------------------------
--- Start: Gamemanager
------------------------------------------------
playRound :: Mode -> String -> Move -> [Cell] -> (Int, Int) -> IO ()
playRound mode msg move field score = do
  drawMainPage field score
  putStrLn msg
  putStrLn (show move ++ "'s turn. Pick a cell (A1 to C3).")
  choice <- prompt "-> "
  let newField = resetWhenKiWon (otherMove move) field
  if isCorrectInput (getFieldIdx choice) newField then do
    let newFieldP = addMove move (getFieldIdx choice) newField
    if isThereAWinner move newFieldP then
      playRound mode (show move ++ " wins!") X (replicate 9 Free) (incrementScore move score)
    else
      if nothingIsFree newFieldP then
        playRound mode "Draw!" X (replicate 9 Free) score
      else
        if mode == PvP then
          playRound mode "" (otherMove move) newFieldP score
        else 
          if mode == PvKI Impossible then do
            let newFieldKi = addMove (otherMove move) (ultraKi (otherMove move) newFieldP) newFieldP
            if isThereAWinner (otherMove move) newFieldKi then
              playRound mode (show (otherMove move) ++ " wins!") X newFieldKi (incrementScore (otherMove move) score)
            else 
              playRound mode "" move newFieldKi score
          else do
            let newFieldKi = addMove (otherMove move) (pickOneFreeCell newFieldP ((fst score) + (snd score))) newFieldP
            if isThereAWinner (otherMove move) newFieldKi then
              playRound mode (show (otherMove move) ++ " wins!") X newFieldKi (incrementScore (otherMove move) score)
            else 
              playRound mode "" move newFieldKi score
  else 
    playRound mode "Wrong input try again." move field score

------------------------------------------------
--- End: Gamemanager
------------------------------------------------