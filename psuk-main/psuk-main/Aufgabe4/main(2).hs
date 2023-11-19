import Data.List (sortBy)
import Data.Ord (comparing)
import System.IO
import System.Process

---------------------------------------------------------------
--- Start: Main
---------------------------------------------------------------
main = do
  file <- readFile "db/cdb.txt"
  let linesOfFile = lines file
  let contacts = loadContacts linesOfFile
  system "clear"
  putStrLn "Welcome to the cool adress management! (for help type 'h')"
  choice <- getChar
  getLine
  managementSystem choice contacts

data Contact = Contact {firstname :: [Char], lastname :: [Char], adress :: [Char], plz :: [Char], town :: [Char], tel :: [Char]}
  deriving (Show)

---------------------------------------------------------------
--- End: Main
---------------------------------------------------------------

---------------------------------------------------------------
--- Start: Load contacts
---------------------------------------------------------------
loadContacts :: [String] -> [Contact]
loadContacts [] = []
loadContacts linesOfFile =
  [ Contact
      (head (getTagContent "Firstname" linesOfFile))
      (head (getTagContent "Lastname" linesOfFile))
      (head (getTagContent "Adress" linesOfFile))
      (head (getTagContent "PLZ" linesOfFile))
      (head (getTagContent "Town" linesOfFile))
      (head (getTagContent "Tel" linesOfFile))
  ]
    ++ loadContacts (nextContact linesOfFile)

nextContact linesOfFile
  | head linesOfFile == "</Contact>" = drop 1 linesOfFile
  | otherwise = nextContact (drop 1 linesOfFile)

getTagContent :: [Char] -> [String] -> [String]
getTagContent tag linesOfFile
  | (head linesOfFile) == ("<" ++ tag ++ ">") = getTagContentHelper ("</" ++ tag ++ ">") (drop 1 linesOfFile)
  | otherwise = getTagContent tag (drop 1 linesOfFile)

getTagContentHelper endTag linesOfFile
  | head linesOfFile == endTag = []
  | otherwise = [head linesOfFile] ++ getTagContentHelper endTag (drop 1 linesOfFile)

---------------------------------------------------------------
--- End: Load contacts
---------------------------------------------------------------

---------------------------------------------------------------
--- Start: Save Contacts
---------------------------------------------------------------
saveContacts :: [Contact] -> IO ()
saveContacts contacts = writeFile "db/cdb.txt" "" >> saveContactsHelper contacts

saveContactsHelper :: [Contact] -> IO ()
saveContactsHelper [] = return ()
saveContactsHelper contacts = do
  appendFile "db/cdb.txt" ("<Contact>\n<Firstname>\n" ++ firstname contact ++ "\n</Firstname>\n<Lastname>\n" ++ lastname contact ++ "\n</Lastname>\n<Adress>\n" ++ adress contact ++ "\n</Adress>\n<PLZ>\n" ++ plz contact ++ "\n</PLZ>\n<Town>\n" ++ town contact ++ "\n</Town>\n<Tel>\n" ++ tel contact ++ "\n</Tel>\n</Contact>\n")
  saveContactsHelper (drop 1 contacts)
  where
    contact = head contacts

---------------------------------------------------------------
--- End: Save Contacts
---------------------------------------------------------------

---------------------------------------------------------------
--- Start: Management System
---------------------------------------------------------------
managementSystem choice contacts
  | choice == 'h' =
    system "clear"
      >> showChoosedCmd "Help"
      >> printHelp
      >> do
        newChoice <- getChar
        getLine
        managementSystem newChoice contacts
  | choice == 's' =
    system "clear"
      >> showChoosedCmd "Show contacts"
      >> do
        showContacts (sortContacts contacts)
        putStrLn "\nType new command (for help type 'h')"
        newChoice <- getChar
        getLine
        managementSystem newChoice contacts
  | choice == 'a' =
    system "clear"
      >> showChoosedCmd "Add contact"
      >> do
        newContacts <- addContact contacts
        putStrLn "\ncontact added..\nType new command (for help type 'h')"
        newChoice <- getChar
        getLine
        managementSystem newChoice newContacts
  | choice == 'f' =
    system "clear"
      >> showChoosedCmd "Search by firstname"
      >> do
        foundContacts <- askAndSearchFirstName contacts
        showContacts foundContacts
        putStrLn "\nType new command (for help type 'h')"
        newChoice <- getChar
        getLine
        managementSystem newChoice contacts
  | choice == 'l' =
    system "clear"
      >> showChoosedCmd "Search by lastname"
      >> do
        foundContacts <- askAndSearchLastName contacts
        showContacts foundContacts
        putStrLn "\nType new command (for help type 'h')"
        newChoice <- getChar
        getLine
        managementSystem newChoice contacts
  | choice == 'd' =
    system "clear"
      >> showChoosedCmd "Delete contact"
      >> do
        newContacts <- askAndRemoveContact contacts
        putStrLn "\ncontact deleted..\nType new command (for help type 'h')"
        newChoice <- getChar
        getLine
        managementSystem newChoice newContacts
  | choice == 'e' =
    system "clear"
      >> showChoosedCmd "Edit contact"
      >> do
        putStrLn "Which contact would you like to edit?"
        newContacts <- askAndRemoveContact contacts
        putStrLn "\nEnter the new data."
        updatedContacts <- addContact newContacts
        putStrLn "\ncontact updated..\nType new command (for help type 'h')"
        newChoice <- getChar
        getLine
        managementSystem newChoice updatedContacts
  | choice == 'q' =
    system "clear"
      >> showChoosedCmd "Quit"
      >> saveContacts (sortContacts contacts)
      >> putStrLn "contacts saved..\nGoodbye!"
  | choice == '\n' = 
      do
        newChoice <- getChar
        getLine
        managementSystem newChoice contacts
  | otherwise =
    putStrLn "\nWrong command! Try again..\nType new command (for help type 'h')"
      >> do
        newChoice <- getChar
        getLine
        managementSystem newChoice contacts

-- where choice = getChar

showChoosedCmd cmd =
  putStrLn
    ( "-----------------------------------\n--- " ++ cmd
        ++ "\n-----------------------------------"
    )

---------------------------------------------------------------
--- End: Management System
---------------------------------------------------------------

---------------------------------------------------------------
--- Start: Functions
---------------------------------------------------------------
printHelp =
  do putStrLn "a: add contact"
    >> putStrLn "d: delete contact"
    >> putStrLn "e: edit contact"
    >> putStrLn "s: show contacts"
    >> putStrLn "f: search by first name"
    >> putStrLn "l: search by last name"
    >> putStrLn "h: help"
    >> putStrLn "q: quit"

showContacts :: [Contact] -> IO ()
showContacts [] = return ()
showContacts contacts = showContact contact >> showContacts (drop 1 contacts)
  where
    contact = head contacts

showContact :: Contact -> IO ()
showContact contact =
  putStr
    ( "Firstname: " ++ (firstname contact) ++ "\n"
        ++ "Lastname : "
        ++ (lastname contact)
        ++ "\n"
        ++ "Adress   : "
        ++ (adress contact)
        ++ "\n"
        ++ "PLZ      : "
        ++ (plz contact)
        ++ "\n"
        ++ "Town     : "
        ++ (town contact)
        ++ "\n"
        ++ "Tel      : "
        ++ (tel contact)
        ++ "\n\n"
    )

addContact contacts = do
  putStrLn "Firstname:"
  aFirstname <- getLine
  putStrLn "Lastname:"
  aLastname <- getLine
  putStrLn "Adress:"
  aAdress <- getLine
  putStrLn "PLZ:"
  aPlz <- getLine
  putStrLn "Town:"
  aTown <- getLine
  putStrLn "Telnum:"
  aTel <- getLine
  return (contacts ++ [Contact aFirstname aLastname aAdress aPlz aTown aTel])

askAndSearchFirstName contacts = do
  putStrLn "Firstname:"
  aFirstname <- getLine
  return (askAndSearchFirstNameHelper aFirstname contacts)

askAndSearchFirstNameHelper :: String -> [Contact] -> [Contact]
askAndSearchFirstNameHelper _ [] = []
askAndSearchFirstNameHelper aFirstname contacts
  | firstname contact == aFirstname = [contact] ++ askAndSearchFirstNameHelper aFirstname (drop 1 contacts)
  | otherwise = askAndSearchFirstNameHelper aFirstname (drop 1 contacts)
  where
    contact = head contacts

askAndSearchLastName contacts = do
  putStrLn "Lastname:"
  aLastname <- getLine
  return (askAndSearchLastNameHelper aLastname contacts)

askAndSearchLastNameHelper :: String -> [Contact] -> [Contact]
askAndSearchLastNameHelper _ [] = []
askAndSearchLastNameHelper aLastname contacts
  | lastname contact == aLastname = [contact] ++ askAndSearchLastNameHelper aLastname (drop 1 contacts)
  | otherwise = askAndSearchLastNameHelper aLastname (drop 1 contacts)
  where
    contact = head contacts

askAndRemoveContact contacts = do
  putStrLn "Firstname:"
  aFirstname <- getLine
  putStrLn "Lastname:"
  aLastname <- getLine
  return (askAndRemoveContactHelper aFirstname aLastname contacts)

askAndRemoveContactHelper _ _ [] = []
askAndRemoveContactHelper aFirstname aLastname contacts
  | firstname contact == aFirstname && lastname contact == aLastname = askAndRemoveContactHelper aFirstname aLastname (drop 1 contacts)
  | otherwise = contact : askAndRemoveContactHelper aFirstname aLastname (drop 1 contacts)
  where
    contact = head contacts

sortContacts contacts = sortBy (comparing firstname) contacts

---------------------------------------------------------------
--- End: Functions
---------------------------------------------------------------
