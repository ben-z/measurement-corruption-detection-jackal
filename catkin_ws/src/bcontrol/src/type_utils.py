from typing import List, Tuple, Union, Any, Optional
import typeguard as _typeguard
from typeguard import checker_lookup_functions, TypeCheckerCallable, TypeCheckMemo, TypeCheckError
from enum import Enum

# Add a checker for enum types
def enum_checker(value: Any, origin_type: Any, args: Tuple[Any,...], memo: TypeCheckMemo) -> None:
    if value not in [e.value for e in origin_type]:
        raise TypeCheckError(f"Expected value of type {origin_type}, got {value}")

def checker_lookup(origin_type: Any, args: Tuple[Any, ...], extras: Tuple[Any, ...]) -> Union[TypeCheckerCallable, None]:
    if issubclass(origin_type, Enum):
        return enum_checker
    return None

# patch the checker lookup functions with custom checkers
checker_lookup_functions.append(checker_lookup)

# re-export typeguard so that users always import from this module.
typeguard = _typeguard
