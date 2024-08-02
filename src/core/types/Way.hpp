#ifndef CHDR_WAY_HPP
#define CHDR_WAY_HPP

namespace CHDR {

    template <typename W = bool>
    class Way {

    private:

        RelationalNode<W> m_Start;
        RelationalNode<W> m_End;

    };

} // CHDR

#endif //CHDR_WAY_HPP